#!/usr/bin/env python3
"""
Bridge STM32 dual ADC (F103) CDC/raw streamed 32-bit packed IQ (ADC1+ADC2) into a FIFO as
interleaved complex samples for GQRX (File I/Q input) or other SDR tools.

Packing (STM32F1 dual regular simultaneous):
        Bits  0..11 : ADC1 result -> I (12-bit right-aligned)
        Bits 12..15 : zero padding
        Bits 16..27 : ADC2 result -> Q (12-bit right-aligned)
        Bits 28..31 : zero padding

Earlier versions mis-decodeds Q at bit 12; fixed to use bit 16.

Output format:
    cf32  : float32 complex (recommended for GQRX) written as contiguous <I,Q> pairs

Recommended for GQRX now: use --format cf32. In GQRX "File" I/Q input dialog select:
    Format: Complex float 32 (if available) OR use external pipe via gnuradio/Soapy for cf32.
If your GQRX build lacks native cf32 file mode, use an external converter to feed cf32 into GQRX.

Typical run for GQRX (220 kHz stream, prebuffer to avoid underrun):
    python host_iq_fifo.py --format cf32 --fifo /tmp/iq_cf32.iq --rate 220000 --prebuf 65536

Then in GQRX:
    1. Set input device to "File" and point to /tmp/iq_cf32.iq
    2. Sample rate: 220000 (or downsample inside GQRX DSP chain)
    3. Disable Loop so it treats growth as live
    4. Start after prebuffer completes

Use --swap-iq-output if downstream expects Q,I ordering.
Use --force-phase 0 in the new pure stream (no stray text) — automatic heuristics can be bypassed.

Ctrl+C to stop; script reports throughput and drop metrics.
"""
import argparse, glob, os, sys, time, math, statistics, stat, fcntl, struct, select, tty, termios
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


def decode_block(buf, out_container, *, zero_i=False, zero_q=False, invert_q=False, phase_correction_deg=0.0):
    """Decode packed 32-bit IQ words into cf32 float32 pairs (little-endian dual12).

    Parameters:
      buf          : bytes-like, length multiple of 4
      out_container: list to which packed '<ff' bytes pairs will be appended
      zero_i/zero_q: if True, force channel to center value (debug)
      invert_q     : invert Q channel polarity around center (debug)
    """
    def lane_words(off):
        b0 = buf[off]; b1 = buf[off+1]; b2 = buf[off+2]; b3 = buf[off+3]
        # little-endian per 16-bit lane: low byte first
        i16 = b0 | (b1 << 8)
        q16 = b2 | (b3 << 8)
        return i16, q16

    pack = struct.pack
    scale = 2048.0
    center = 2048
    # precompute rotation for phase correction
    if phase_correction_deg:
        rad = phase_correction_deg * (math.pi/180.0)
        rot_cos = math.cos(rad)
        rot_sin = math.sin(rad)
    else:
        rot_cos = 1.0; rot_sin = 0.0
    for off in range(0, len(buf), 4):
        i16, q16 = lane_words(off)
        i_raw = i16 & 0x0FFF
        q_raw = q16 & 0x0FFF
        if zero_i: i_raw = center
        if zero_q: q_raw = center
        if invert_q and not zero_q:
            q_raw = (center*2 - q_raw)
        fi = (i_raw - center) / scale
        fq = (q_raw - center) / scale
        # apply phase correction rotation to complex sample (I + jQ)
        if rot_sin != 0.0:
            ri = fi * rot_cos - fq * rot_sin
            rq = fi * rot_sin + fq * rot_cos
        else:
            ri = fi; rq = fq
        out_container.append(pack('<ff', ri, rq))

# analyze_samples removed per request: alignment/metric diagnostics and spectral helpers disabled.


# estimate_image_alpha removed: image-alpha estimation (FFT-based spectral helpers) disabled per request.


# refine_image_alpha removed: iterative spectral refinement disabled per request.

## __main__ guard moved to end of file. Duplicate doc block removed.

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


"""(Removed duplicate decode_block implementation.)"""

# analyze_samples removed: alignment/metric diagnostics disabled


# estimate_image_alpha removed: spectral helpers disabled


# refine_image_alpha removed: iterative spectral refinement disabled

# spectral_phase_select removed: FFT-based spectral alignment helper disabled per request.


def main():
    ap = argparse.ArgumentParser(description='CDC -> FIFO 8-bit I/Q bridge for GQRX')
    ap.add_argument('--port', help='Explicit serial port path')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--fifo', default='iq_fifo.iq', help='FIFO path (created if absent)')
    ap.add_argument('--capture', help='Optional file to also dump FIFO output (cf32)')
    ap.add_argument('--timeout', type=float, default=0.01, help='Serial read timeout seconds')
    ap.add_argument('--prebuf', type=int, default=0, help='Bytes to accumulate before writing to FIFO (startup cushion)')
    ap.add_argument('--chunk', type=int, default=4096, help='Serial read chunk size')
    ap.add_argument('--skip-stat', action='store_true', help='(Legacy) Skip printable ASCII chunks containing STAT (OFF by default; may cause misalignment if binary matches)')
    ap.add_argument('--format', choices=['cf32'], default='cf32', help='Output sample format for FIFO (cf32 only)')
    ap.add_argument('--zero-i', action='store_true', help='Force I channel to zero (debug)')
    ap.add_argument('--zero-q', action='store_true', help='Force Q channel to zero (debug)')
    ap.add_argument('--analyze', type=int, metavar='N', help='Capture first N 32-bit words, analyze, then continue streaming')
    ap.add_argument('--dump-words', action='store_true', help='With --analyze, also print the raw 32-bit words in hex')
    ap.add_argument('--dump-tail', type=int, metavar='N', help='Keep last N 32-bit raw words and print them on exit (helps inspect end of stream / after corruption)')
    ap.add_argument('--raw-capture', metavar='FILE', help='Capture ALL raw incoming bytes (pre-decoding) to FILE for offline inspection (CAUTION: grows indefinitely)')
    # map mode forced to 'dual12' (STM32F1 dual regular simultaneous layout)
    # Alignment/auto-realign options removed to keep the host deterministic
    # Initial alignment is assumed correct; manual numeric phase discard (0..3)
    ap.add_argument('--log-phase', type=int, metavar='N', help='Every N samples, log current block start offset (diagnostic)')
    # bias-phase0 removed (legacy tie-break behavior removed)
    # (spectral-align and spectral-align-test options removed)
    ap.add_argument('--exit-after-analyze', action='store_true', help='Perform analysis block then continue streaming unless --force-exit-after-analyze also given')
    ap.add_argument('--force-exit-after-analyze', action='store_true', help='With --exit-after-analyze, actually exit after analysis (legacy behavior)')
    ap.add_argument('--analyze-before-fifo', action='store_true', help='Perform --analyze / --dump-words before opening FIFO (avoids waiting for reader)')
    # IRR/image-rejection related options removed per request (measure_irr, irr-*, tone-*)
    ap.add_argument('--probe-layout', type=int, metavar='N', help='Capture N words and display multiple interpretations (endianness, lane positions)')
    # Removed legacy/diagnostic options: swap-lanes, big-endian-words, invert-q,
    # swap-iq-output, dc-remove/dc-alpha, image-cancel related options,
    # periodic-sync-block and related IRR-after-cancel options.
    # alpha-neighbors (image-cancel stability) removed
    ap.add_argument('--dump-first-hex', type=int, metavar='N', help='Dump first N raw serial bytes in hex (before alignment) and keep streaming')
    ap.add_argument('--diag', action='store_true', help='Enable runtime diagnostics: log serial chunk sizes, timing gaps, small-packet counts (helps debug ticking/underrun)')
    ap.add_argument('--diag-only', action='store_true', help='Diagnostics-only: do not open FIFO or write decoded output (isolates serial timing from FIFO backpressure)')
    ap.add_argument('--measure-skew', action='store_true', help='Continuously measure I/Q skew at a specified tone frequency and print (deg, ns)')
    ap.add_argument('--tone-hz', type=float, default=7000.0, help='Tone frequency in Hz used for skew measurement (default 7000)')
    ap.add_argument('--fs', type=float, default=180000.0, help='Sampling rate in Hz (default 180000). Adjust to your board/sample rate')
    ap.add_argument('--skew-interval', type=float, default=1.0, help='Minimum seconds between skew reports (default 1.0s)')
    ap.add_argument('--skew-correction-deg', type=float, default=0.0, help='Apply a fixed phase correction (degrees) to complex samples before writing FIFO (positive rotates I+jQ by +deg)')
    ap.add_argument('--skew-correction-ns', type=float, default=-2000.0, help='Apply a fractional delay (ns) to Q channel (positive delays Q)')
    ap.add_argument('--skew-step-ns', type=float, default=1.0, help='Step size in ns when adjusting skew interactively (default 1.0 ns)')
    ap.add_argument('--trace-phase', action='store_true', help='Log block-start byte offset (mod 4) and report changes (diagnostic)')
    ap.add_argument('--lock-start-phase', action='store_true', help='Lock initial byte phase after startup -- ignore later phase-change requests')
    ap.add_argument('--initial-discard', type=int, default=3, help='Discard N leading bytes from stream at startup (useful to adjust byte-phase). Values are taken modulo 4.')
    # periodic alignment check removed (no periodic alignment diagnostics)
    # (strip-sync-frame removed: firmware no longer inserts periodic sync frames)
    args = ap.parse_args()
    if args.initial_discard:
        print(f"[phase] initial-discard requested: {int(args.initial_discard) % 4} byte(s)")
    # Removed legacy CLI flags and auto-realignment handling to keep the host
    # stream simple and deterministic. The host now assumes correct framing
    # and only provides manual numeric phase discard and diagnostic tracing.

    # (spectral-align-test removed)
    # Convenience: allow --dump-words alone; default to analyzing first 64 words
    if args.dump_words and not args.analyze:
        args.analyze = 64
        print(f"[info] --dump-words specified without --analyze; defaulting to analyze first {args.analyze} words", file=sys.stderr)
    # If user asked only for probe pre-FIFO, auto-set analyze length
    if args.probe_layout and args.analyze_before_fifo and not args.analyze:
        args.analyze = args.probe_layout
        print(f"[info] --probe-layout implies --analyze {args.analyze} pre-FIFO", file=sys.stderr)

    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=args.timeout)
    if args.dump_first_hex:
        pre_hex = ser.read(args.dump_first_hex)
        print(f"[dump-first-hex] {len(pre_hex)} bytes: {' '.join(f'{b:02X}' for b in pre_hex)}")
        # Reinject these bytes into initial alignment path
        globals()['_prefetched_initial_bytes'] = pre_hex

    # Early analysis path BEFORE FIFO open if requested
    if args.analyze and args.analyze_before_fifo:
        target_bytes = args.analyze * 4
        buf = bytearray()
        print(f"[pre-analyze] Capturing {args.analyze} words ({target_bytes} bytes) before FIFO open...")
        # IRR pre-analyze checks removed
        while len(buf) < target_bytes:
            chunk = ser.read(args.chunk)
            if '_prefetched_initial_bytes' in globals():
                chunk = globals().pop('_prefetched_initial_bytes') + chunk
            if not chunk:
                continue
            buf.extend(chunk)
        # (spectral alignment removed) no phase_shift applied here
        phase_shift = 0
        # Ensure we still have full target_bytes after shift; if not, pull more bytes or pad failure.
        if phase_shift:
            # Attempt to read extra bytes to compensate for discarded leading bytes
            while len(buf) < target_bytes + phase_shift:
                chunk = ser.read(args.chunk)
                if '_prefetched_initial_bytes' in globals():
                    chunk = globals().pop('_prefetched_initial_bytes') + chunk
                if not chunk:
                    break
                buf.extend(chunk)
        aligned_full_len = target_bytes
        if len(buf) - phase_shift < target_bytes:
            aligned_full_len = len(buf) - phase_shift - (len(buf) - phase_shift) % 4
            print(f"[pre-analyze] Warning: insufficient bytes after shift; using {aligned_full_len//4} words instead of {args.analyze}.")
        aligned = buf[phase_shift:phase_shift+aligned_full_len]
        actual_words = len(aligned)//4
        if actual_words != args.analyze:
            print(f"[pre-analyze] Adjusted analyze word count: {actual_words}")
        words = []
        for off in range(0, actual_words*4, 4):
            # Always interpret incoming stream as little-endian 32-bit words
            w = aligned[off] | (aligned[off+1] << 8) | (aligned[off+2] << 16) | (aligned[off+3] << 24)
            words.append(w)
        if args.dump_words:
            print("RAW WORDS:")
            for i,w in enumerate(words):
                print(f" {i:04d}: 0x{w:08X}")
        # analyze_samples removed: alignment/metric diagnostics disabled
        # IRR pre-analyze measurement removed
        if args.probe_layout:
            probe_layout(words[:args.probe_layout])
        if args.exit_after_analyze and args.force_exit_after_analyze:
            print("[pre-analyze] Force-exit requested after analysis.")
            ser.close()
            return
        # continue streaming -> fall through to FIFO setup

    fifo_fd = None
    fifo_path = args.fifo
    if not args.diag_only:
        fifo_path = args.fifo
        ensure_fifo(fifo_path)
        print(f"FIFO ready: {fifo_path} (waiting for reader when opened) format={args.format}")

        # Open FIFO for write only; on macOS need reader first or open blocks.
        while fifo_fd is None:
            try:
                fifo_fd = os.open(fifo_path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                if args.analyze_before_fifo and args.analyze and args.exit_after_analyze and args.force_exit_after_analyze:
                    # Already analyzed and user wants to exit; break early
                    print("[info] Analysis complete and force-exit requested; skipping FIFO wait.")
                    ser.close()
                    return
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
    # Diagnostics state (enabled with --diag)
    last_chunk_time = None
    gap_threshold = 0.01  # seconds; gaps larger than this are suspicious (10 ms)
    diag_last_summary = time.time()
    diag_chunk_count = 0
    diag_small_chunk_count = 0
    diag_total_bytes = 0
    # Runtime phase control
    bytes_processed = 0  # total bytes consumed from stream (for phase arithmetic)
    pending_discard = int(args.initial_discard) % 4  # bytes to discard to achieve requested phase (0..3)
    current_phase = 0
    term_orig = None
    term_fd = None
    term_has_cb = False
    # (DC removal and image-cancel features removed — stream is passed through)
    # initial alignment state removed
    # Skew measurement state
    last_skew_time = 0.0
    # Trace/lock diagnostics
    locked_phase = None
    last_traced_phase = None
    # Track last reported skew correction so interactive changes are visible
    last_reported_skew_correction = None
    # Q-history buffer for fractional-delay skew correction
    q_history = deque()
    max_q_history = 8192
    # (Periodic sync-frame removal state removed; firmware no longer injects frames)

    # Alignment/phase-metric functions removed: host will assume phase0 framing and not auto-realign.



    try:
        # If running in an interactive terminal, enable cbreak mode to capture single key presses
        try:
            if sys.stdin.isatty():
                term_fd = sys.stdin.fileno()
                term_orig = termios.tcgetattr(term_fd)
                tty.setcbreak(term_fd)
                term_has_cb = True
                print('[info] Phase control: press 0/1/2/3 to set byte-phase while running')
        except Exception:
            term_has_cb = False

        while True:
            chunk = ser.read(args.chunk)
            if '_prefetched_initial_bytes' in globals():
                chunk = globals().pop('_prefetched_initial_bytes') + chunk
            now = time.time()
            if not chunk:
                # no data returned in this read; treat as a short gap
                if args.diag:
                    if last_chunk_time is not None:
                        gap = now - last_chunk_time
                        if gap > gap_threshold:
                            print(f"[diag] NO-CHUNK gap: {gap*1000:.1f} ms since last chunk")
                continue
            # diagnostics: timestamp, chunk size, detect gap
            if args.diag:
                if last_chunk_time is not None:
                    gap = now - last_chunk_time
                    if gap > gap_threshold:
                        print(f"[diag] GAP: {gap*1000:.1f} ms between serial chunks (len={len(chunk)})")
                last_chunk_time = now
                diag_chunk_count += 1
                diag_total_bytes += len(chunk)
                if len(chunk) < 32:
                    diag_small_chunk_count += 1
                # per-second summary
                if now - diag_last_summary >= 1.0:
                    avg_bps = diag_total_bytes / max(1.0, now - diag_last_summary)
                    print(f"[diag-summary] chunks={diag_chunk_count} small_chunks={diag_small_chunk_count} bytes_last_sec={diag_total_bytes} avg_bps~{avg_bps:.0f}")
                    # reset per-second counters
                    diag_chunk_count = 0
                    diag_small_chunk_count = 0
                    diag_total_bytes = 0
                    diag_last_summary = now
            # (Periodic sync-frame stripping removed; firmware no longer injects frames)
            if args.skip_stat:
                printable = sum(32 <= b < 127 for b in chunk)
                if printable/len(chunk) > ASCII_SKIP_THRESHOLD and b'STAT' in chunk:
                    continue
            if raw_capture_fh:
                raw_capture_fh.write(chunk)

            # Check for user keypresses (non-blocking). If user pressed 0..3, schedule phase change.
            if term_has_cb:
                try:
                    r,_,_ = select.select([sys.stdin], [], [], 0)
                    if r:
                        # read up to 3 bytes to capture arrow key sequences like '\x1b[A' (up) or '\x1b[B' (down)
                        chs = sys.stdin.read(3)
                        if not chs:
                            continue
                        # numeric phase control (0..3)
                        for ch in chs:
                            if ch in ('0','1','2','3'):
                                desired = int(ch)
                                if locked_phase is not None:
                                    print(f"[phase] Locked to {locked_phase}; ignoring requested phase change to {desired}")
                                else:
                                    need = (desired - (bytes_processed % 4)) % 4
                                    pending_discard = need
                                    print(f"[phase] Requested phase={desired} (will discard {need} byte(s) when available)")
                                break
                        # arrow keys: ESC [ C = right, ESC [ D = left
                        # Right arrow increases fractional skew (ns), Left arrow decreases it.
                        if chs.startswith('\x1b[C') or chs.startswith('\x1b\x1b[C'):
                            args.skew_correction_ns = float(args.skew_correction_ns) + float(args.skew_step_ns)
                            print(f"[skew] increased delay -> {args.skew_correction_ns:.3f} ns")
                        elif chs.startswith('\x1b[D') or chs.startswith('\x1b\x1b[D'):
                            args.skew_correction_ns = float(args.skew_correction_ns) - float(args.skew_step_ns)
                            print(f"[skew] decreased delay -> {args.skew_correction_ns:.3f} ns")
                        # also allow + and - keys for adjustment (phase rotation)
                        if '+' in chs:
                            args.skew_correction_deg = float(args.skew_correction_deg) + 1.0
                            print(f"[skew] increased correction -> {args.skew_correction_deg:.3f} deg")
                        if '-' in chs:
                            args.skew_correction_deg = float(args.skew_correction_deg) - 1.0
                            print(f"[skew] decreased correction -> {args.skew_correction_deg:.3f} deg")
                        # (Left/Right arrows now control skew ns; '['/']' keys removed)
                except Exception:
                    # if stdin not selectable, ignore
                    pass

            # (Periodic sync-block detection removed)

            # Initial alignment and auto-realign logic removed — assume stream is phase-0 and aligned.
            # Always append incoming chunk directly to partial (no auto-shifting performed).
            partial += chunk

            # If a phase-change discard is pending, try to apply it now (if enough bytes buffered)
            if pending_discard:
                if len(partial) >= pending_discard:
                    # Drop the requested number of bytes from partial to align framing
                    partial = partial[pending_discard:]
                    bytes_processed += pending_discard
                    current_phase = (current_phase + pending_discard) % 4
                    print(f"[phase] Applied discard of {pending_discard} byte(s). New byte offset mod4 = {bytes_processed % 4}")
                    pending_discard = 0
                else:
                    # wait for more bytes to arrive
                    continue

            # (Initial alignment logic moved earlier; this block intentionally removed)

            # auto-realign behavior is disabled; no initial alignment performed
            # auto-realign removed: host will not attempt to auto-shift stream
            # Trace and optional lock of start-phase (before we cut full words)
            start_phase = bytes_processed % 4
            if args.trace_phase:
                if start_phase != last_traced_phase:
                    print(f"[trace] block_start_offset_mod4={start_phase} bytes_processed={bytes_processed} partial_len={len(partial)}")
                    last_traced_phase = start_phase
            if args.lock_start_phase and locked_phase is None:
                locked_phase = start_phase
                print(f"[phase] Locked start-phase to {locked_phase} (will ignore later phase change requests)")

            if len(partial) < 4:
                continue
            cut = len(partial) - (len(partial) % 4)
            block = partial[:cut]
            partial = partial[cut:]
            bytes_processed += cut
            # Optional analysis path (first N words)
            if args.analyze and not analyzed:
                for off in range(0, len(block), 4):
                    if len(analyze_words) < args.analyze:
                        # Interpret as little-endian 32-bit words
                        w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                        analyze_words.append(w)
                if len(analyze_words) >= args.analyze:
                    if args.dump_words:
                        print("RAW WORDS:")
                for i,w in enumerate(analyze_words):
                    print(f" {i:04d}: 0x{w:08X}")
                # analyze_samples removed: alignment/metric diagnostics disabled
                    if args.probe_layout:
                        probe_layout(analyze_words[:args.probe_layout])
                    analyzed = True
                    if args.exit_after_analyze and args.force_exit_after_analyze:
                        print('[stream-analyze] Force-exit requested after analysis inside streaming path. Exiting.')
                        break
            # Tail capture path (last N words across entire session)
            if tail_words is not None:
                for off in range(0, len(block), 4):
                    w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                    tail_words.append(w)
            # (Skew measurement moved to after correction so reported values reflect
            # the residual skew after rotation/fractional-delay have been applied.)
            # Only cf32 output supported
            # If the user requested a fractional-delay skew correction (ns), decode into
            # float lists and apply the fractional delay to Q before packing.
            if abs(float(args.skew_correction_ns)) > 0.0:
                # Report changes in skew-correction value
                if last_reported_skew_correction is None or float(args.skew_correction_ns) != float(last_reported_skew_correction):
                    print(f"[skew] applying fractional delay {float(args.skew_correction_ns):.3f} ns")
                    last_reported_skew_correction = float(args.skew_correction_ns)
                I_list, Q_list = decode_block_to_lists(block, zero_i=args.zero_i, zero_q=args.zero_q, invert_q=False, phase_correction_deg=args.skew_correction_deg)
                outbuf = apply_fractional_delay_and_pack(I_list, Q_list, float(args.skew_correction_ns), args.fs, q_history, max_q_history)
            else:
                pieces = []
                # Report skew-correction degrees when it changes so user can observe effect
                if last_reported_skew_correction is None or float(args.skew_correction_deg) != float(last_reported_skew_correction):
                    print(f"[skew] applying rotation {float(args.skew_correction_deg):.3f} deg")
                    last_reported_skew_correction = float(args.skew_correction_deg)
                decode_block(block, pieces, zero_i=args.zero_i, zero_q=args.zero_q, invert_q=False, phase_correction_deg=args.skew_correction_deg)
                # Pass-through stream: no DC removal or image-cancel processing
                outbuf = b''.join(pieces)
            # Measure skew AFTER any correction has been applied so printed values
            # reflect the residual skew seen by the downstream (cf32) stream.
            if args.measure_skew:
                now = time.time()
                if now - last_skew_time >= args.skew_interval:
                    res = measure_skew_from_cf32(outbuf, args.fs, args.tone_hz)
                    if res is not None:
                        deg, ns, magI, magQ = res
                        print(f"[skew] tone={args.tone_hz}Hz phase_diff={deg:.3f} deg skew={ns:.1f} ns magI={magI:.3f} magQ={magQ:.3f}")
                    last_skew_time = now

            total_samples += len(outbuf)//8  # 8 bytes per complex sample
            if not args.diag_only:
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
            # Periodic alignment check removed
            # Periodic progress every ~2 seconds
            if total_samples and (total_samples % (DEFAULT_RATE*2) == 0):
                elapsed = time.time() - start_time
                rate = total_samples/elapsed if elapsed>0 else 0.0
                print(f"Samples={total_samples}  OutBytes={total_bytes_out}  Rate={rate:.1f} sps ({rate/1000:.1f} ksps) fmt={args.format}")
            # IRR streaming capture removed
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if capture_fh:
            capture_fh.close()
        if raw_capture_fh:
            raw_capture_fh.close()
        # Restore terminal state if we enabled cbreak/raw mode
        try:
            if term_has_cb and term_orig is not None and term_fd is not None:
                termios.tcsetattr(term_fd, termios.TCSADRAIN, term_orig)
        except Exception:
            pass
        # Only close FIFO if we actually opened one (diag-only skips opening)
        try:
            if fifo_fd is not None:
                os.close(fifo_fd)
        except Exception:
            pass
        elapsed = time.time()-start_time
        if elapsed>0:
            print(f"Final: samples={total_samples} avgRate={total_samples/elapsed:.1f} sps")
        # sync-frame stripping removed
        if tail_words is not None and len(tail_words):
            print(f"TAIL RAW WORDS (last {len(tail_words)}):")
            base_index = total_samples - len(tail_words)
            for i,w in enumerate(tail_words):
                print(f" {base_index + i:08d}: 0x{w:08X}")

# validate_alignment removed: alignment validation/auto-realign is disabled in the simplified host

# IRR / image-rejection helpers removed per request

def probe_layout(words, *, swap_lanes=False, big_endian=False):
    """Print a brief layout interpretation of first <=16 words."""
    count = min(len(words), 16)
    print(f"[probe-layout] showing {count} words swap_lanes={swap_lanes}")
    for i in range(count):
        w = words[i]
        lo16 = w & 0xFFFF
        hi16 = (w >> 16) & 0xFFFF
        i12 = w & 0x0FFF
        q12 = (w >> 16) & 0x0FFF
        if swap_lanes:
            i_out, q_out = q12, i12
        else:
            i_out, q_out = i12, q12
        print(f" {i:02d}: w=0x{w:08X} lo16=0x{lo16:04X} hi16=0x{hi16:04X} I12={i_out:03X} Q12={q_out:03X}")


def decode_block_to_lists(buf, *, zero_i=False, zero_q=False, invert_q=False, phase_correction_deg=0.0):
    """Decode packed 32-bit words into two Python lists of floats (I_list, Q_list).

    This mirrors decode_block but returns numeric lists instead of packed bytes so we can
    apply fractional-delay skew correction to Q before packing.
    """
    I = []
    Q = []
    scale = 2048.0
    center = 2048
    if phase_correction_deg:
        rad = phase_correction_deg * (math.pi/180.0)
        rot_cos = math.cos(rad)
        rot_sin = math.sin(rad)
    else:
        rot_cos = 1.0; rot_sin = 0.0
    off = 0
    for off in range(0, len(buf), 4):
        b0 = buf[off]; b1 = buf[off+1]; b2 = buf[off+2]; b3 = buf[off+3]
        i16 = b0 | (b1 << 8)
        q16 = b2 | (b3 << 8)
        i_raw = i16 & 0x0FFF
        q_raw = q16 & 0x0FFF
        if zero_i: i_raw = center
        if zero_q: q_raw = center
        if invert_q and not zero_q:
            q_raw = (center*2 - q_raw)
        fi = (i_raw - center) / scale
        fq = (q_raw - center) / scale
        if rot_sin != 0.0:
            ri = fi * rot_cos - fq * rot_sin
            rq = fi * rot_sin + fq * rot_cos
        else:
            ri = fi; rq = fq
        I.append(ri)
        Q.append(rq)
    return I, Q


def apply_fractional_delay_and_pack(I, Q, skew_ns, fs, q_history, max_q_history):
    """Apply fractional delay (ns) to Q channel and pack interleaved cf32 bytes.

    Uses linear interpolation across combined q_history + Q to produce delayed Q samples.
    Positive skew_ns delays Q (i.e., Q_out[n] = Q_in[n - d]).
    """
    if not Q:
        return b''
    # If no skew requested, pack directly
    if abs(skew_ns) < 1e-12:
        pack = struct.pack
        out = bytearray()
        for ri, rq in zip(I, Q):
            out.extend(pack('<ff', ri, rq))
        # update q_history
        q_history.extend(Q)
        while len(q_history) > max_q_history:
            q_history.popleft()
        return bytes(out)

    # compute fractional sample delay
    d = float(skew_ns) * float(fs) / 1e9
    combined_q = list(q_history) + Q
    N_hist = len(q_history)
    NQ = len(Q)
    out = bytearray()
    pack = struct.pack
    for n in range(NQ):
        src = n + N_hist - d
        k = int(math.floor(src))
        frac = src - k
        if k < 0:
            k = 0; frac = 0.0
        if k+1 >= len(combined_q):
            qk = combined_q[-1]
            qk1 = combined_q[-1]
        else:
            qk = combined_q[k]
            qk1 = combined_q[k+1]
        q_out = (1.0 - frac) * qk + frac * qk1
        out.extend(pack('<ff', I[n], q_out))

    # update q_history with newest Q samples
    q_history.extend(Q)
    while len(q_history) > max_q_history:
        q_history.popleft()
    return bytes(out)


def measure_skew_from_raw(block, fs, f0):
    """Return (phase_diff_deg, skew_ns, magI, magQ) measured from raw 32-bit packed IQ words in block.

    block: bytes with length multiple of 4 (little-endian 32-bit words)
    fs: sampling rate (Hz)
    f0: tone frequency (Hz)
    Method: accumulate C = sum_n x[n] * exp(-j*2*pi*f0*n/fs) for I and Q separately.
    """
    import math
    N = len(block) // 4
    if N <= 0:
        return None
    delta = 2.0 * math.pi * f0 / fs
    # rot = exp(-j*delta)
    rot_r = math.cos(delta)
    rot_i = -math.sin(delta)
    pr = 1.0; pi = 0.0
    CI_r = CI_i = CQ_r = CQ_i = 0.0
    center = 2048
    scale = 2048.0
    off = 0
    for n in range(N):
        b0 = block[off]; b1 = block[off+1]; b2 = block[off+2]; b3 = block[off+3]
        off += 4
        i16 = b0 | (b1 << 8)
        q16 = b2 | (b3 << 8)
        i_raw = i16 & 0x0FFF
        q_raw = q16 & 0x0FFF
        fi = (i_raw - center) / scale
        fq = (q_raw - center) / scale
        CI_r += fi * pr; CI_i += fi * pi
        CQ_r += fq * pr; CQ_i += fq * pi
        # rotate phasor by rot (pr + j pi) *= rot_r + j rot_i
        tmp_pr = pr * rot_r - pi * rot_i
        tmp_pi = pr * rot_i + pi * rot_r
        pr, pi = tmp_pr, tmp_pi

    angleI = math.atan2(CI_i, CI_r)
    angleQ = math.atan2(CQ_i, CQ_r)
    phase_diff = angleQ - angleI
    # normalize to [-pi, pi]
    while phase_diff <= -math.pi: phase_diff += 2*math.pi
    while phase_diff > math.pi: phase_diff -= 2*math.pi
    # skew in seconds
    skew_s = phase_diff / (2.0 * math.pi * f0) if f0 != 0 else 0.0
    skew_ns = skew_s * 1e9
    magI = math.hypot(CI_r, CI_i)
    magQ = math.hypot(CQ_r, CQ_i)
    return (phase_diff * 180.0 / math.pi, skew_ns, magI, magQ)


def measure_skew_from_cf32(cf32_bytes, fs, f0):
    """Measure skew from cf32 bytes (interleaved <float32 I, float32 Q>).

    Returns (phase_diff_deg, skew_ns, magI, magQ) or None on error.
    """
    import struct, math
    if not cf32_bytes:
        return None
    N = len(cf32_bytes) // 8
    if N <= 0:
        return None
    delta = 2.0 * math.pi * f0 / fs
    rot_r = math.cos(delta)
    rot_i = -math.sin(delta)
    pr = 1.0; pi = 0.0
    CI_r = CI_i = CQ_r = CQ_i = 0.0
    off = 0
    for n in range(N):
        # unpack I (float32) then Q (float32)
        ri = struct.unpack_from('<f', cf32_bytes, off)[0]; off += 4
        rq = struct.unpack_from('<f', cf32_bytes, off)[0]; off += 4
        CI_r += ri * pr; CI_i += ri * pi
        CQ_r += rq * pr; CQ_i += rq * pi
        # rotate phasor by rot
        tmp_pr = pr * rot_r - pi * rot_i
        tmp_pi = pr * rot_i + pi * rot_r
        pr, pi = tmp_pr, tmp_pi

    angleI = math.atan2(CI_i, CI_r)
    angleQ = math.atan2(CQ_i, CQ_r)
    phase_diff = angleQ - angleI
    # normalize to [-pi, pi]
    while phase_diff <= -math.pi: phase_diff += 2*math.pi
    while phase_diff > math.pi: phase_diff -= 2*math.pi
    skew_s = phase_diff / (2.0 * math.pi * f0) if f0 != 0 else 0.0
    skew_ns = skew_s * 1e9
    magI = math.hypot(CI_r, CI_i)
    magQ = math.hypot(CQ_r, CQ_i)
    return (phase_diff * 180.0 / math.pi, skew_ns, magI, magQ)

# Ensure script executes main() when run directly.
if __name__ == '__main__':
    main()
