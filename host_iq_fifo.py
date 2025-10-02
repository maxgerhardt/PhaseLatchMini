#!/usr/bin/env python3
"""
Bridge STM32 dual ADC (F103) CDC/raw streamed 32-bit packed IQ (ADC1+ADC2) into a FIFO as
interleaved I/Q for GQRX (File I/Q input) or other SDR tools.

STM32F1 dual regular simultaneous mode packs samples as:
    Bits  0..11 : ADC1 result (right-aligned 12-bit)  -> I
    Bits 12..15 : zero padding
    Bits 16..27 : ADC2 result (right-aligned 12-bit)  -> Q
    Bits 28..31 : zero padding

Previously this script incorrectly assumed the Q sample started at bit 12. Fixed to use bit 16.

Workflow:
 1. Create FIFO automatically (default: iq_fifo.iq). If it exists and is a pipe, reuse.
 2. Open CDC serial (/dev/tty.usbmodem*) and continuously read raw bytes.
 3. Parse little-endian 32-bit words; extract I = (w & 0x0FFF), Q = ((w >> 16) & 0x0FFF).
 4. Downscale to unsigned 8-bit via shift-right 4 (0..4095 -> 0..255).
 5. Write interleaved [I0,Q0,I1,Q1,...] to FIFO (unbuffered) in one of:
     s8   : signed 8-bit (-128..127) I/Q (recommended for GQRX "Signed 8-bit I/Q")
     u8   : unsigned 8-bit (legacy, not ideal for GQRX which expects signed)
     raw16: little-endian unsigned 16-bit pairs (debug/inspection)
     cf32 : float32 complex (I then Q)
 6. Optionally write a secondary file capture (raw 8-bit) for offline analysis.

GQRX Setup (File I/Q mode):
    - Sample rate: 140000
  - Format: Unsigned 8-bit I/Q (interleaved IQIQ...)
  - Uncheck 'Loop' so it treats as live; start after script begins writing.

If GQRX struggles to follow FIFO growth, run with --prebuf to accumulate some kB before GQRX start.

Usage:
  python host_iq_fifo.py --fifo iq_fifo.iq
  python host_iq_fifo.py --fifo iq_fifo.iq --capture out8.iq --prebuf 32768

Ctrl+C to stop; script reports counts and average throughput.
"""
import argparse, glob, os, sys, time, math, statistics, stat, fcntl, struct
from collections import deque

try:
    import serial
except ImportError:
    print('ERROR: pyserial required (pip install pyserial)', file=sys.stderr)
    sys.exit(2)

DEFAULT_RATE = 140000  # complex samples/second (for info only)

ASCII_SKIP_THRESHOLD = 0.90  # skip chunk if mostly printable text and contains 'STAT'


def find_port(explicit=None):
    if explicit:
        return explicit
    ports = sorted(glob.glob('/dev/tty.usbmodem*'))
    if not ports:
        raise SystemExit('No /dev/tty.usbmodem* device found')
    return ports[0]

def ensure_fifo(path):
    if os.path.exists(path):
        st_mode = os.stat(path).st_mode
        if not stat.S_ISFIFO(st_mode):
            raise SystemExit(f'Path {path} exists and is not a FIFO')
    else:
        os.mkfifo(path)


def decode_block(buf, fmt, out_container, *, zero_i=False, zero_q=False, map_mode='dual12'):
    """Decode packed 32-bit IQ words into desired format.

    Packing (STM32F1 dual regular simultaneous):
      Bits  0..11 : ADC1 result (I), bits 12..15 zero
      Bits 16..27 : ADC2 result (Q), bits 28..31 zero

    Parameters:
      buf          : bytes-like, length multiple of 4
      fmt          : 'u8' (8-bit interleaved) or 'cf32' (float32 I,Q)
      out_container: bytearray for 'u8' or list for 'cf32'
      zero_i/q     : force chosen channel to 0 (debug isolation)
    """
    if map_mode not in ('dual12','split16'):
        raise ValueError('map_mode must be dual12 or split16')

    def lane_words(off):
        b0 = buf[off]; b1 = buf[off+1]; b2 = buf[off+2]; b3 = buf[off+3]
        # Fixed little-endian per 16-bit lane
        i16 = b0 | (b1 << 8)
        q16 = b2 | (b3 << 8)
        return i16, q16

    if fmt == 'u8':
        ba = out_container
        for off in range(0, len(buf), 4):
            i16, q16 = lane_words(off)
            if map_mode == 'dual12':
                i_val = i16 & 0x0FFF
                q_val = q16 & 0x0FFF
                if zero_i: i_val = 0
                if zero_q: q_val = 0
                ba.append(i_val >> 4)
                ba.append(q_val >> 4)
            else:  # split16
                if zero_i: i16 = 0x8000
                if zero_q: q16 = 0x8000
                ba.append(i16 >> 8)
                ba.append(q16 >> 8)
    elif fmt == 's8':
        # Signed 8-bit output expected by many SDR tools (center at 0)
        ba = out_container
        for off in range(0, len(buf), 4):
            i16, q16 = lane_words(off)
            if map_mode == 'dual12':
                i_raw = i16 & 0x0FFF
                q_raw = q16 & 0x0FFF
                center = 2048; scale_shift = 4
            else:
                i_raw = i16
                q_raw = q16
                center = 32768; scale_shift = 8
            if zero_i: i_raw = center
            if zero_q: q_raw = center
            i_s = (i_raw - center) >> scale_shift  # arithmetic since Python int
            q_s = (q_raw - center) >> scale_shift
            # Clamp to signed 8-bit just in case
            if i_s < -128: i_s = -128
            elif i_s > 127: i_s = 127
            if q_s < -128: q_s = -128
            elif q_s > 127: q_s = 127
            # Convert to unsigned storage for bytearray
            ba.append(i_s & 0xFF)
            ba.append(q_s & 0xFF)
    elif fmt == 'raw16':
        # Emit little-endian 16-bit unsigned I then Q without scaling (helps inspect real dynamic range)
        ba = out_container
        for off in range(0, len(buf), 4):
            i16, q16 = lane_words(off)
            if map_mode == 'dual12':
                i_val = i16 & 0x0FFF
                q_val = q16 & 0x0FFF
            else:
                i_val = i16
                q_val = q16
            if zero_i: i_val = (2048 if map_mode=='dual12' else 0x8000)
            if zero_q: q_val = (2048 if map_mode=='dual12' else 0x8000)
            # store as 16-bit unsigned little endian
            ba.append(i_val & 0xFF); ba.append((i_val >> 8) & 0xFF)
            ba.append(q_val & 0xFF); ba.append((q_val >> 8) & 0xFF)
    else:  # cf32
        pack = struct.pack
        scale = 2048.0 if map_mode=='dual12' else 32768.0
        center = 2048 if map_mode=='dual12' else 32768
        for off in range(0, len(buf), 4):
            i16, q16 = lane_words(off)
            if map_mode == 'dual12':
                i_raw = i16 & 0x0FFF
                q_raw = q16 & 0x0FFF
            else:
                i_raw = i16
                q_raw = q16
            if zero_i: i_raw = center
            if zero_q: q_raw = center
            fi = (i_raw - center) / scale
            fq = (q_raw - center) / scale
            out_container.append(pack('<ff', fi, fq))

def analyze_samples(words):
    """Given a list of 32-bit raw words, extract candidate I/Q interpretations and print diagnostics.

    Hypotheses:
      H1: I = bits0..11, Q = bits16..27 (expected dual-pack, sparse zeros in padding bits).
      H2: I = lower16, Q = upper16 treated as independent 16-bit samples (if full 16-bit path used).
      H3: I = Q (duplicate) indicating we are reading only ADC1 and upper 16 replicate lower 16.

    We compute correlation and equality metrics to guide debugging.
    """
    import math
    i_h1 = []
    q_h1 = []
    lo16 = []
    hi16 = []
    same_count = 0
    for w in words:
        i1 = w & 0x0FFF
        q1 = (w >> 16) & 0x0FFF
        i_h1.append(i1)
        q_h1.append(q1)
        l16 = w & 0xFFFF
        h16 = (w >> 16) & 0xFFFF
        lo16.append(l16)
        hi16.append(h16)
        if l16 == h16:
            same_count += 1
    def stats(v):
        n = len(v)
        if n == 0:
            return (0,0,0)
        mean = sum(v)/n
        var = sum((x-mean)**2 for x in v)/n if n>0 else 0
        return (mean, math.sqrt(var), min(v), max(v))
    def corr(a,b):
        n = len(a)
        if n==0: return 0.0
        ma = sum(a)/n; mb = sum(b)/n
        num = sum((x-ma)*(y-mb) for x,y in zip(a,b))
        da = math.sqrt(sum((x-ma)**2 for x in a))
        db = math.sqrt(sum((y-mb)**2 for y in b))
        return num/(da*db) if da>0 and db>0 else 0.0
    mean_i, std_i, min_i, max_i = stats(i_h1)
    mean_q, std_q, min_q, max_q = stats(q_h1)
    mean_l, std_l, min_l, max_l = stats(lo16)
    mean_h, std_h, min_h, max_h = stats(hi16)
    print("ANALYZE: Samples=%d" % len(words))
    print(f" H1 12-bit: I(mean={mean_i:.1f} std={std_i:.1f} min={min_i} max={max_i})  Q(mean={mean_q:.1f} std={std_q:.1f} min={min_q} max={max_q})  corr={corr(i_h1,q_h1):.3f}")
    print(f" 16-bit view: LO(mean={mean_l:.1f} std={std_l:.1f} range={min_l}-{max_l}) HI(mean={mean_h:.1f} std={std_h:.1f} range={min_h}-{max_h}) equal_pairs={same_count/len(words)*100:.1f}%")
    # Heuristic suggestions
    if same_count/len(words) > 95.0:
        print(" Suggestion: Upper 16 bits mirror lower 16 -> likely only ADC1 data duplicated (check dual mode or DMA alignment).")
    if abs(corr(i_h1,q_h1)) > 0.95 and same_count/len(words) < 95.0:
        print(" High I/Q correlation: possible leakage or identical channel source.")
    if std_q < 2 and std_i > 10:
        print(" Q channel nearly flat: verify extraction shift (maybe we are masking zeros).")



def main():
    ap = argparse.ArgumentParser(description='CDC -> FIFO 8-bit I/Q bridge for GQRX')
    ap.add_argument('--port', help='Explicit serial port path')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--fifo', default='iq_fifo.iq', help='FIFO path (created if absent)')
    ap.add_argument('--capture', help='Optional file to also dump 8-bit IQ stream')
    ap.add_argument('--timeout', type=float, default=0.01, help='Serial read timeout seconds')
    ap.add_argument('--prebuf', type=int, default=0, help='Bytes to accumulate before writing to FIFO (startup cushion)')
    ap.add_argument('--chunk', type=int, default=4096, help='Serial read chunk size')
    ap.add_argument('--skip-stat', action='store_true', help='(Legacy) Skip printable ASCII chunks containing STAT (OFF by default; may cause misalignment if binary matches)')
    ap.add_argument('--format', choices=['s8','u8','cf32','raw16'], default='cf32', help='Output sample format for FIFO (default s8)')
    ap.add_argument('--zero-i', action='store_true', help='Force I channel to zero (debug)')
    ap.add_argument('--zero-q', action='store_true', help='Force Q channel to zero (debug)')
    ap.add_argument('--analyze', type=int, metavar='N', help='Capture first N 32-bit words, analyze, then continue streaming')
    ap.add_argument('--dump-words', action='store_true', help='With --analyze, also print the raw 32-bit words in hex')
    ap.add_argument('--dump-tail', type=int, metavar='N', help='Keep last N 32-bit raw words and print them on exit (helps inspect end of stream / after corruption)')
    ap.add_argument('--raw-capture', metavar='FILE', help='Capture ALL raw incoming bytes (pre-decoding) to FILE for offline inspection (CAUTION: grows indefinitely)')
    ap.add_argument('--map', choices=['dual12','split16'], default='dual12', help='Interpretation of 32-bit word layout (default dual12)')
    ap.add_argument('--validate-alignment', action='store_true', help='Heuristically verify 32-bit framing before streaming')
    ap.add_argument('--validate-samples', type=int, default=256, help='Word count used for alignment validation (default 256)')
    ap.add_argument('--auto-realign', action='store_true', help='Continuously monitor for byte misalignment and auto-correct (dual12 mode only)')
    ap.add_argument('--realign-window', type=int, default=512, help='Byte window used for realignment scoring (default 512)')
    ap.add_argument('--realign-threshold', type=float, default=0.06, help='Minimum score advantage required to trigger realignment (default 0.06)')
    ap.add_argument('--realign-min-zero', type=float, default=0.80, help='If current phase zero-high ratio drops below this, consider realignment (default 0.80)')
    ap.add_argument('--realign-confirm', type=int, default=3, help='Number of consecutive windows a non-zero phase must dominate before shifting (hysteresis, default 3)')
    ap.add_argument('--realign-min-best-zero', type=float, default=0.90, help='Require best candidate zero-high ratio >= this before shifting (default 0.90)')
    ap.add_argument('--no-initial-align', action='store_true', help='Do not perform initial phase sniff before streaming (default: perform)')
    ap.add_argument('--initial-align-samples', type=int, default=256, help='Word count collected for initial alignment scoring (default 256)')
    ap.add_argument('--log-phase', type=int, metavar='N', help='Every N samples, log current alignment scores (diagnostic)')
    ap.add_argument('--force-phase', type=int, choices=[0,1,2,3], help='Force an initial byte phase (0..3) instead of auto detection')
    ap.add_argument('--prefer-phase', type=int, choices=[0,1,2,3], help='Preferred phase if scores within margin (tie-break)')
    ap.add_argument('--prefer-margin', type=float, default=0.01, help='Score difference threshold to allow choosing preferred phase (default 0.01)')
    args = ap.parse_args()

    # Convenience: allow --dump-words alone; default to analyzing first 64 words
    if args.dump_words and not args.analyze:
        args.analyze = 64
        print(f"[info] --dump-words specified without --analyze; defaulting to analyze first {args.analyze} words", file=sys.stderr)

    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=args.timeout)

    # Lazy FIFO open (block until a reader connects)
    fifo_path = args.fifo
    ensure_fifo(fifo_path)
    print(f"FIFO ready: {fifo_path} (waiting for reader when opened) format={args.format}")

    # Open FIFO for write only; on macOS need reader first or open blocks.
    # Workaround: open in non-blocking and retry until a reader appears.
    fifo_fd = None
    while fifo_fd is None:
        try:
            fifo_fd = os.open(fifo_path, os.O_WRONLY | os.O_NONBLOCK)
        except OSError:
            print("Waiting for reader (start GQRX File I/Q)...")
            time.sleep(0.5)
    # Switch to blocking
    fl = fcntl.fcntl(fifo_fd, fcntl.F_GETFL)
    fcntl.fcntl(fifo_fd, fcntl.F_SETFL, fl & ~os.O_NONBLOCK)

    capture_fh = open(args.capture,'wb') if args.capture else None
    raw_capture_fh = open(args.raw_capture,'wb') if args.raw_capture else None
    tail_words = deque(maxlen=args.dump_tail) if args.dump_tail else None

    partial = b''
    total_samples = 0
    total_bytes_out = 0
    start_time = time.time()
    prebuf_store = bytearray()

    analyzed = False
    analyze_words = []
    raw_accum = bytearray() if args.validate_alignment else None
    alignment_done = False
    realign_buffer = bytearray()
    last_realign_at_samples = 0
    initial_alignment_done = args.no_initial_align
    initial_align_buffer = bytearray()
    candidate_phase = None
    candidate_count = 0

    def compute_alignment_scores(buf, words_target):
        phases = []
        length = len(buf)
        for phase in range(4):
            usable = length - phase
            if usable < 4:
                phases.append((phase, -1.0, 0.0))
                continue
            words = min(words_target, usable // 4)
            if words == 0:
                phases.append((phase, -1.0, 0.0))
                continue
            minv = 0xFFFF; maxv = 0; hi_zero = 0
            for i in range(words):
                off = phase + i*4
                b0 = buf[off]; b1 = buf[off+1]
                val = (b0 | (b1<<8)) & 0x0FFF
                if val < minv: minv = val
                if val > maxv: maxv = val
                if (b1 & 0xF0) == 0:
                    hi_zero += 1
            span = maxv - minv
            ratio_zero_hi = hi_zero/words
            score = (span / 4095.0) * 0.7 + ratio_zero_hi * 0.3
            phases.append((phase, score, ratio_zero_hi))
        return phases

    def compute_phase_metrics(buf, words_target):
        """Enhanced metrics per candidate phase.
        Returns list of dicts: {phase, words, span, zero_hi_b1, zero_hi_b3, zero_both_ratio, out_of_range, score}
        zero_hi_b1: fraction of lane1 high bytes (b1) whose upper nibble is 0
        zero_hi_b3: fraction of lane2 high bytes (b3) whose upper nibble is 0
        zero_both_ratio: fraction where BOTH high bytes have upper nibble 0 (proxy for good dual12 alignment)
        out_of_range: fraction of 12-bit values (>0x0FFF after masking) observed (should be 0 for correct alignment)
        score: weighted aggregate emphasizing zero_both_ratio and low out_of_range.
        """
        metrics = []
        length = len(buf)
        for phase in range(4):
            usable = length - phase
            if usable < 4:
                metrics.append({'phase':phase,'words':0,'score':-1,'zero_hi_b1':0,'zero_hi_b3':0,'zero_both_ratio':0,'span':0,'out_of_range':1.0})
                continue
            words = min(words_target, usable//4)
            if words == 0:
                metrics.append({'phase':phase,'words':0,'score':-1,'zero_hi_b1':0,'zero_hi_b3':0,'zero_both_ratio':0,'span':0,'out_of_range':1.0})
                continue
            minv = 0xFFFF; maxv = 0
            z1 = z2 = both = oor = 0
            for i in range(words):
                off = phase + i*4
                b0 = buf[off]; b1 = buf[off+1]; b2 = buf[off+2]; b3 = buf[off+3]
                v1 = (b0 | (b1<<8)) & 0xFFFF
                v2 = (b2 | (b3<<8)) & 0xFFFF
                # 12-bit masked
                m1 = v1 & 0x0FFF
                if m1 < minv: minv = m1
                if m1 > maxv: maxv = m1
                if (b1 & 0xF0) == 0: z1 += 1
                if (b3 & 0xF0) == 0: z2 += 1
                if (b1 & 0xF0) == 0 and (b3 & 0xF0) == 0: both += 1
                if (v1 & 0xF000) and (v1 & 0xF000) != 0: # coarse check for non-zero upper nibble; indicates right-align not preserved
                    pass
                # detect out-of-range when interpreting supposed lane values as 12-bit
                if (v1 & 0xF000) != 0 or (v2 & 0xF000) != 0:
                    oor += 1
            span = maxv - minv
            zero_hi_b1 = z1/words
            zero_hi_b3 = z2/words
            both_ratio = both/words
            oor_frac = oor/(words*2)
            # scoring: favor high both_ratio, low oor, reasonable span (> small threshold) to ensure not flat
            span_norm = min(span/4095.0, 1.0)
            score = both_ratio*0.6 + zero_hi_b1*0.1 + zero_hi_b3*0.1 + span_norm*0.2 - oor_frac*0.5
            metrics.append({'phase':phase,'words':words,'score':score,'zero_hi_b1':zero_hi_b1,'zero_hi_b3':zero_hi_b3,'zero_both_ratio':both_ratio,'span':span,'out_of_range':oor_frac})
        return metrics


    try:
        while True:
            chunk = ser.read(args.chunk)
            if not chunk:
                continue
            if args.skip_stat:
                printable = sum(32 <= b < 127 for b in chunk)
                if printable/len(chunk) > ASCII_SKIP_THRESHOLD and b'STAT' in chunk:
                    continue
            if raw_capture_fh:
                raw_capture_fh.write(chunk)

            # Before initial alignment completes, accumulate only in initial_align_buffer (not in partial)
            if not initial_alignment_done and args.map=='dual12':
                initial_align_buffer.extend(chunk)
                # Forced phase: act immediately once we have at least force_phase bytes
                if args.force_phase is not None and len(initial_align_buffer) >= (args.force_phase + 4):
                    shift = args.force_phase
                    partial = initial_align_buffer[shift:]
                    realign_buffer.extend(initial_align_buffer)  # feed history
                    print(f"[initial-align] Forced phase {shift} (discarded {shift} byte(s)).")
                    initial_alignment_done = True
                elif args.force_phase is None:
                    needed = args.initial_align_samples * 4
                    if len(initial_align_buffer) >= needed:
                        metrics = compute_phase_metrics(initial_align_buffer[:needed], args.initial_align_samples)
                        # Re-score emphasizing zero_both_ratio dominance
                        # Already computed score includes both_ratio heavy weight; we still explicitly pick by both_ratio then score
                        # Determine best candidate(s)
                        # Primary key: zero_both_ratio, secondary: score
                        sorted_m = sorted(metrics, key=lambda m: (m['zero_both_ratio'], m['score']), reverse=True)
                        best = sorted_m[0]
                        shift = best['phase']
                        # Tie / preference handling
                        if args.prefer_phase is not None:
                            # Find preferred phase metrics
                            pref = next((m for m in metrics if m['phase']==args.prefer_phase), None)
                            if pref:
                                score_diff = (best['score'] - pref['score'])
                                z_diff = (best['zero_both_ratio'] - pref['zero_both_ratio'])
                                if (pref['zero_both_ratio'] > 0) and (score_diff <= args.prefer_margin) and (z_diff <= 0 or z_diff <= 0.02):
                                    # Adopt preferred phase
                                    shift = pref['phase']
                                    best = pref
                                    print(f"[initial-align] Prefer-phase override -> phase {shift} (score diff={score_diff:.3f} zDiff={z_diff*100:.1f}%)")
                        else:
                            # Deterministic tie-break: among phases whose zero_both_ratio within 0.5% and score within prefer-margin, pick smallest phase number
                            close = [m for m in metrics if abs(m['zero_both_ratio']-best['zero_both_ratio']) <= 0.005 and abs(m['score']-best['score']) <= args.prefer_margin]
                            if len(close) > 1:
                                chosen = min(close, key=lambda m: m['phase'])
                                if chosen['phase'] != best['phase']:
                                    print(f"[initial-align] Tie-break: phases {[c['phase'] for c in close]} similar -> selecting lowest phase {chosen['phase']}")
                                    best = chosen; shift = chosen['phase']
                        if shift != 0:
                            partial = initial_align_buffer[shift:]
                            print(f"[initial-align] Selected phase {shift} (discarded {shift} byte(s)). both={best['zero_both_ratio']*100:.1f}% score={best['score']:.3f} span={best['span']}")
                        else:
                            partial = initial_align_buffer
                            print(f"[initial-align] Selected phase 0. both={best['zero_both_ratio']*100:.1f}% score={best['score']:.3f} span={best['span']}")
                        tbl = ' '.join(f"p{m['phase']}:{m['zero_both_ratio']*100:.0f}%/{m['score']:.3f}" for m in metrics)
                        print(f"[initial-align-metrics] {tbl}")
                        realign_buffer.extend(initial_align_buffer)
                        initial_alignment_done = True
                # Until alignment done, skip rest of loop
                if not initial_alignment_done:
                    continue
            else:
                # Alignment already done or not applicable -> append chunk to partial
                partial += chunk

            # (Initial alignment logic moved earlier; this block intentionally removed)

            if args.auto_realign and args.map == 'dual12':
                realign_buffer.extend(chunk)
                # Keep buffer bounded
                if len(realign_buffer) > args.realign_window + 4:
                    del realign_buffer[:len(realign_buffer) - (args.realign_window + 4)]
                if len(realign_buffer) >= args.realign_window:
                    # Evaluate current phase quality
                    scores = compute_alignment_scores(realign_buffer[-args.realign_window:], min(args.realign_window//4, 128))
                    phase0 = next(s for s in scores if s[0]==0)
                    phase_best = max(scores, key=lambda x: x[1])
                    # Decide if misaligned: low zero-high ratio OR another phase significantly better
                    # Hysteresis logic: track candidate phase dominance
                    advantage = (phase_best[1]-phase0[1]) if phase_best[0] != 0 else 0.0
                    trigger = False
                    if phase_best[0] == 0:
                        candidate_phase = None
                        candidate_count = 0
                    else:
                        if (phase0[2] < args.realign_min_zero and advantage >= args.realign_threshold and phase_best[2] >= args.realign_min_best_zero) or \
                           (advantage >= (args.realign_threshold*1.5) and phase_best[2] >= args.realign_min_best_zero):
                            if candidate_phase == phase_best[0]:
                                candidate_count += 1
                            else:
                                candidate_phase = phase_best[0]
                                candidate_count = 1
                            if candidate_count >= args.realign_confirm:
                                trigger = True
                        else:
                            # reset if conditions not met
                            if candidate_phase == phase_best[0]:
                                candidate_count = 0
                    if trigger:
                        shift = phase_best[0]
                        if shift <= len(partial):
                            partial = partial[shift:]
                            realign_buffer = realign_buffer[shift:]
                            print(f"[auto-realign] Shifted stream by {shift} byte(s) at sample={total_samples} (phase0 score={phase0[1]:.3f} zeroHi={phase0[2]*100:.1f}% -> new phase {phase_best[0]} score={phase_best[1]:.3f} zeroHi={phase_best[2]*100:.1f}% ConfSeq={candidate_count})")
                            last_realign_at_samples = total_samples
                            candidate_phase = None
                            candidate_count = 0
                    elif phase_best[0] != 0 and phase_best[2] >= args.realign_min_best_zero and advantage >= args.realign_threshold:
                        # Verbose candidate log (single line) for visibility
                        if args.log_phase:
                            print(f"[auto-realign-cand] phase{phase_best[0]} adv={advantage:.3f} zeroHi={phase_best[2]*100:.1f}% seq={candidate_count+1}/{args.realign_confirm}")
                    if args.log_phase and total_samples and (total_samples % args.log_phase == 0):
                        dbg = ' '.join(f"p{p}:{s:.3f}/{zh*100:.1f}%" for p,s,zh in scores)
                        print(f"[phase] samples={total_samples} {dbg}")
            if raw_accum is not None and not alignment_done:
                raw_accum.extend(chunk)
                if args.validate_alignment and len(raw_accum) >= args.validate_samples * 4:
                    validate_alignment(raw_accum[:args.validate_samples*4], args.validate_samples)
                    alignment_done = True
                continue
            if len(partial) < 4:
                continue
            cut = len(partial) - (len(partial) % 4)
            block = partial[:cut]
            partial = partial[cut:]
            # Optional analysis path (first N words)
            if args.analyze and not analyzed:
                for off in range(0, len(block), 4):
                    if len(analyze_words) < args.analyze:
                        w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                        analyze_words.append(w)
                if len(analyze_words) >= args.analyze:
                    if args.dump_words:
                        print("RAW WORDS:")
                        for i,w in enumerate(analyze_words):
                            print(f" {i:04d}: 0x{w:08X}")
                    analyze_samples(analyze_words)
                    analyzed = True
            # Tail capture path (last N words across entire session)
            if tail_words is not None:
                for off in range(0, len(block), 4):
                    w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                    tail_words.append(w)
            if args.format in ('u8','s8'):
                outbuf = bytearray()
                decode_block(block, args.format, outbuf, zero_i=args.zero_i, zero_q=args.zero_q, map_mode=args.map)
                total_samples += len(outbuf)//2
            elif args.format == 'raw16':
                outbuf = bytearray()
                decode_block(block, 'raw16', outbuf, zero_i=args.zero_i, zero_q=args.zero_q, map_mode=args.map)
                total_samples += len(outbuf)//4
            else:  # cf32
                pieces = []
                decode_block(block, 'cf32', pieces, zero_i=args.zero_i, zero_q=args.zero_q, map_mode=args.map)
                outbuf = b''.join(pieces)
                total_samples += len(outbuf)//8  # 8 bytes per complex sample
            if args.prebuf and len(prebuf_store) < args.prebuf:
                prebuf_store.extend(outbuf)
                if len(prebuf_store) >= args.prebuf:
                    try:
                        os.write(fifo_fd, prebuf_store)
                    except BrokenPipeError:
                        print("FIFO reader closed during prebuffer write; exiting.")
                        break
                    total_bytes_out += len(prebuf_store)
                    prebuf_store.clear()
                continue
            try:
                os.write(fifo_fd, outbuf)
            except BrokenPipeError:
                print("FIFO reader closed; exiting.")
                break
            total_bytes_out += len(outbuf)
            if capture_fh:
                capture_fh.write(outbuf)
            # Periodic progress every ~2 seconds
            if total_samples and (total_samples % (DEFAULT_RATE*2) == 0):
                elapsed = time.time() - start_time
                rate = total_samples/elapsed if elapsed>0 else 0.0
                print(f"Samples={total_samples}  OutBytes={total_bytes_out}  Rate={rate:.1f} sps ({rate/1000:.1f} ksps) fmt={args.format}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if capture_fh:
            capture_fh.close()
        if raw_capture_fh:
            raw_capture_fh.close()
        os.close(fifo_fd)
        elapsed = time.time()-start_time
        if elapsed>0:
            print(f"Final: samples={total_samples} avgRate={total_samples/elapsed:.1f} sps")
        if tail_words is not None and len(tail_words):
            print(f"TAIL RAW WORDS (last {len(tail_words)}):")
            base_index = total_samples - len(tail_words)
            for i,w in enumerate(tail_words):
                print(f" {base_index + i:08d}: 0x{w:08X}")

if __name__ == '__main__':
    main()

def validate_alignment(raw_bytes, words_target):
    """Score all 4 possible starting byte phases for 32-bit word framing.

    For a correctly aligned dual-ADC 12-bit right-aligned stream (little endian 32-bit words):
      - Each 4-byte word = [I_low, I_high, Q_low, Q_high], where the channel value is in bits 11..0 of its 16-bit lane.
      - If the signal hovers near mid-scale (around 0x800) the HIGH BYTE (I_high or Q_high) will often be 0x07 or 0x08.
        Seeing many 0x07/0x08 bytes is expected for mid-scale noise and does NOT indicate misalignment.
      - Misalignment usually produces values >0x0FFF after masking, or strong correlation between adjacent reconstructed samples.

    Heuristic:
      For each candidate starting byte phase (0..3), interpret successive 4-byte groups as words and:
        * Measure span of (word & 0x0FFF) for first lane
        * Count fraction of high bytes whose upper nibble is 0 (optional indicator of unused high nibble)
      Combine into a score. Highest score suggests most plausible alignment. Phase 0 is the expected one.
    """
    phases = []
    length = len(raw_bytes)
    for phase in range(4):
        usable = length - phase
        if usable < 4:
            phases.append((phase, -1.0, 'insufficient'))
            continue
        words = min(words_target, usable // 4)
        minv = 0xFFFF
        maxv = 0
        hi_zero = 0
        for i in range(words):
            off = phase + i*4
            b0 = raw_bytes[off]
            b1 = raw_bytes[off+1]
            # b2,b3 not needed for basic score, but could be included
            # b2 = raw_bytes[off+2]; b3 = raw_bytes[off+3]
            val = (b0 | (b1 << 8)) & 0x0FFF
            if val < minv: minv = val
            if val > maxv: maxv = val
            if (b1 & 0xF0) == 0: hi_zero += 1
        span = maxv - minv
        ratio_zero_hi = hi_zero / words if words else 0
        score = (span / 4095.0) * 0.7 + ratio_zero_hi * 0.3
        phases.append((phase, score, f"span={span} zeroHi%={ratio_zero_hi*100:.1f}"))
    best = max(phases, key=lambda x: x[1])
    print("ALIGN VALIDATION")
    for phase, score, desc in phases:
        mark = '<<' if phase == best[0] else '  '
        print(f" phase {phase}: score={score:.3f} {mark} {desc}")
    if best[0] == 0:
        print(" Alignment consistent. Frequent 0x07/0x08 high bytes = mid-scale 12-bit samples (normal).")
    else:
        print(f" WARNING: Best phase suggests offset {best[0]} bytes. If downstream display looks wrong, discard {best[0]} leading byte(s) before decoding.")
