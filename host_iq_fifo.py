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

Output formats:
    s8    : signed 8-bit (center 0) interleaved I,Q
    u8    : unsigned 8-bit (legacy)
    raw16 : little-endian unsigned 16-bit I then Q
    cf32  : float32 complex (recommended for GQRX) written as contiguous <I,Q> pairs

Recommended for GQRX now: use --format cf32. In GQRX "File" I/Q input dialog select:
    Format: Complex float 32 (if available) OR use external pipe via gnuradio/Soapy for cf32.
If your GQRX build lacks native cf32 file mode, you can still choose Signed 8-bit I/Q and run with --format s8.

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


def decode_block(buf, fmt, out_container, *, zero_i=False, zero_q=False, map_mode='dual12', invert_q=False):
    """Decode packed 32-bit IQ words into desired format.

    Packing (STM32F1 dual regular simultaneous):
      Bits  0..11 : ADC1 result (I), bits 12..15 zero
      Bits 16..27 : ADC2 result (Q), bits 28..31 zero

        Parameters:
            buf          : bytes-like, length multiple of 4
            fmt          : 'u8', 's8', 'raw16', or 'cf32'
            out_container: bytearray for 'u8'/'s8'/'raw16' or list for 'cf32'
            zero_i/q     : force chosen channel to DC center (debug isolation)
            invert_q     : invert Q channel polarity (around its center) to test quadrature sign convention
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
                i_raw = i16 & 0x0FFF
                q_raw = q16 & 0x0FFF
                if zero_i: i_raw = 2048
                if zero_q: q_raw = 2048
                if invert_q and not zero_q:
                    # reflect around center (2048) so (q_raw - 2048) -> -(q_raw - 2048)
                    q_raw = 4096 - q_raw
                ba.append(i_raw >> 4)
                ba.append(q_raw >> 4)
            else:  # split16
                i_raw = i16
                q_raw = q16
                if zero_i: i_raw = 0x8000
                if zero_q: q_raw = 0x8000
                if invert_q and not zero_q:
                    q_raw = (0x10000 - q_raw)  # reflect around 0x8000
                ba.append(i_raw >> 8)
                ba.append(q_raw >> 8)
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
            if invert_q and not zero_q:
                # Invert around center -> 2*center - q_raw
                q_raw = (center*2 - q_raw)
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
                center16 = 2048
            else:
                i_val = i16
                q_val = q16
                center16 = 0x8000
            if zero_i: i_val = center16
            if zero_q: q_val = center16
            if invert_q and not zero_q:
                q_val = (center16*2 - q_val)
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
            if invert_q and not zero_q:
                q_raw = (center*2 - q_raw)
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


def estimate_image_alpha(words, *, fs, map_mode='dual12'):
    """Estimate complex image cancellation coefficient alpha from a set of raw words.

    Approach:
      1. Form complex samples (centered, normalized) like in IRR measurement.
      2. FFT using numpy; locate main positive-frequency bin (exclude DC).
      3. Image bin is mirrored negative frequency (N-k).
      4. alpha ≈ spec[image_bin] / spec[main_bin]. (Complex ratio capturing amplitude & phase.)

    Returns complex alpha or None if estimation fails.
    """
    try:
        import numpy as np, math
    except Exception:
        return None
    N = len(words)
    if N < 64:
        return None
    I=[]; Q=[]
    if map_mode=='dual12':
        center=2048.0; scale=2048.0
        for w in words:
            I.append((w & 0x0FFF) - center)
            Q.append(((w >> 16) & 0x0FFF) - center)
    else:
        center=32768.0; scale=32768.0
        for w in words:
            I.append((w & 0xFFFF) - center)
            Q.append(((w >> 16) & 0xFFFF) - center)
    c = np.array([ (I[i]/scale) + 1j*(Q[i]/scale) for i in range(N) ], dtype=np.complex128)
    spec = np.fft.fft(c)
    mag2 = np.abs(spec)**2
    df = fs / N
    start_bin = 1
    end_bin = (N//2) - 1
    main_bin = None; main_pow=0.0
    for k in range(start_bin, end_bin+1):
        p = mag2[k]
        if p > main_pow:
            main_pow=p; main_bin=k
    if main_bin is None or main_pow <= 0:
        return None
    image_bin = (N - main_bin) % N
    # Optional neighbor integration for alpha computation
    alpha_neighbors = 0
    if 'last_parsed_args' in globals():
        alpha_neighbors = getattr(globals()['last_parsed_args'], 'alpha_neighbors', 0) or 0
    def integrate_bin(center):
        if alpha_neighbors <= 0:
            return spec[center]
        lo = max(0, center - alpha_neighbors)
        hi = min(len(spec)-1, center + alpha_neighbors)
        return spec[lo:hi+1].sum() / (hi - lo + 1)
    main_val = integrate_bin(main_bin)
    image_val = integrate_bin(image_bin)
    if abs(main_val) < 1e-12:
        return None
    alpha = image_val / main_val
    return alpha


def refine_image_alpha(words, fs, map_mode, iterations=3):
    """Iteratively refine image cancellation alpha by applying cancellation and re-estimating.

    Returns refined alpha (complex) or 0+0j if estimation fails.
    """
    base_alpha = estimate_image_alpha(words, fs=fs, map_mode=map_mode)
    if base_alpha is None:
        return 0+0j
    current_words = words[:]
    for _ in range(max(1, iterations)):
        # Apply current alpha to produce corrected words then re-estimate.
        try:
            import numpy as np
        except Exception:
            return base_alpha
        # Reconstruct complex samples
        I=[]; Q=[]; center = (2048 if map_mode=='dual12' else 32768); scale=center
        for w in current_words:
            if map_mode=='dual12':
                I.append((w & 0x0FFF) - center)
                Q.append(((w>>16)&0x0FFF) - center)
            else:
                I.append((w & 0xFFFF) - center)
                Q.append(((w>>16)&0xFFFF) - center)
        c = np.array([(I[i]/scale)+1j*(Q[i]/scale) for i in range(len(current_words))], dtype=np.complex128)
        c_corr = c - base_alpha * np.conjugate(c)
        # Re-pack into words
        new_words = []
        if map_mode=='dual12':
            for s in c_corr:
                i_val = int((s.real*scale + center)) & 0x0FFF
                q_val = int((s.imag*scale + center)) & 0x0FFF
                new_words.append(i_val | (q_val<<16))
        else:
            for s in c_corr:
                i_val = int((s.real*scale + center)) & 0xFFFF
                q_val = int((s.imag*scale + center)) & 0xFFFF
                new_words.append(i_val | (q_val<<16))
        current_words = new_words
        new_alpha = estimate_image_alpha(current_words, fs=fs, map_mode=map_mode)
        if new_alpha is not None:
            base_alpha = new_alpha
    return base_alpha

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


def decode_block(buf, fmt, out_container, *, zero_i=False, zero_q=False, map_mode='dual12', invert_q=False):
    """Decode packed 32-bit IQ words into desired format.

    Packing (STM32F1 dual regular simultaneous):
      Bits  0..11 : ADC1 result (I), bits 12..15 zero
      Bits 16..27 : ADC2 result (Q), bits 28..31 zero

        Parameters:
            buf          : bytes-like, length multiple of 4
            fmt          : 'u8', 's8', 'raw16', or 'cf32'
            out_container: bytearray for 'u8'/'s8'/'raw16' or list for 'cf32'
            zero_i/q     : force chosen channel to DC center (debug isolation)
            invert_q     : invert Q channel polarity (around its center) to test quadrature sign convention
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
                i_raw = i16 & 0x0FFF
                q_raw = q16 & 0x0FFF
                if zero_i: i_raw = 2048
                if zero_q: q_raw = 2048
                if invert_q and not zero_q:
                    # reflect around center (2048) so (q_raw - 2048) -> -(q_raw - 2048)
                    q_raw = 4096 - q_raw
                ba.append(i_raw >> 4)
                ba.append(q_raw >> 4)
            else:  # split16
                i_raw = i16
                q_raw = q16
                if zero_i: i_raw = 0x8000
                if zero_q: q_raw = 0x8000
                if invert_q and not zero_q:
                    q_raw = (0x10000 - q_raw)  # reflect around 0x8000
                ba.append(i_raw >> 8)
                ba.append(q_raw >> 8)
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
            if invert_q and not zero_q:
                # Invert around center -> 2*center - q_raw
                q_raw = (center*2 - q_raw)
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
                center16 = 2048
            else:
                i_val = i16
                q_val = q16
                center16 = 0x8000
            if zero_i: i_val = center16
            if zero_q: q_val = center16
            if invert_q and not zero_q:
                q_val = (center16*2 - q_val)
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
            if invert_q and not zero_q:
                q_raw = (center*2 - q_raw)
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


def estimate_image_alpha(words, *, fs, map_mode='dual12'):
    """Estimate complex image cancellation coefficient alpha from a set of raw words.

    Approach:
      1. Form complex samples (centered, normalized) like in IRR measurement.
      2. FFT using numpy; locate main positive-frequency bin (exclude DC).
      3. Image bin is mirrored negative frequency (N-k).
      4. alpha ≈ spec[image_bin] / spec[main_bin]. (Complex ratio capturing amplitude & phase.)

    Returns complex alpha or None if estimation fails.
    """
    try:
        import numpy as np, math
    except Exception:
        return None
    N = len(words)
    if N < 64:
        return None
    I=[]; Q=[]
    if map_mode=='dual12':
        center=2048.0; scale=2048.0
        for w in words:
            I.append((w & 0x0FFF) - center)
            Q.append(((w >> 16) & 0x0FFF) - center)
    else:
        center=32768.0; scale=32768.0
        for w in words:
            I.append((w & 0xFFFF) - center)
            Q.append(((w >> 16) & 0xFFFF) - center)
    c = np.array([ (I[i]/scale) + 1j*(Q[i]/scale) for i in range(N) ], dtype=np.complex128)
    spec = np.fft.fft(c)
    mag2 = np.abs(spec)**2
    df = fs / N
    start_bin = 1
    end_bin = (N//2) - 1
    main_bin = None; main_pow=0.0
    for k in range(start_bin, end_bin+1):
        p = mag2[k]
        if p > main_pow:
            main_pow=p; main_bin=k
    if main_bin is None or main_pow <= 0:
        return None
    image_bin = (N - main_bin) % N
    # Optional neighbor integration for alpha computation
    alpha_neighbors = 0
    if 'last_parsed_args' in globals():
        alpha_neighbors = getattr(globals()['last_parsed_args'], 'alpha_neighbors', 0) or 0
    def integrate_bin(center):
        if alpha_neighbors <= 0:
            return spec[center]
        lo = max(0, center - alpha_neighbors)
        hi = min(len(spec)-1, center + alpha_neighbors)
        return spec[lo:hi+1].sum() / (hi - lo + 1)
    main_val = integrate_bin(main_bin)
    image_val = integrate_bin(image_bin)
    if abs(main_val) < 1e-12:
        return None
    alpha = image_val / main_val
    return alpha


def refine_image_alpha(words, fs, map_mode, iterations=3):
    """Iteratively refine image cancellation coefficient alpha.

    Performs multiple passes estimating alpha = image/main then applying cancellation
    to a synthetic complex sequence. Returns cumulative alpha.
    """
    try:
        import numpy as np
    except Exception:
        return 0+0j
    base_alpha = 0+0j
    current_words = list(words)
    for it in range(iterations):
        est = estimate_image_alpha(current_words, fs=fs, map_mode=map_mode)
        if est is None:
            break
        base_alpha += est
        # Build complex samples from current_words
        I=[]; Q=[]
        if map_mode=='dual12':
            center=2048.0; scale=2048.0
            for w in current_words:
                I.append((w & 0x0FFF)-center)
                Q.append(((w>>16)&0x0FFF)-center)
            max_val = 4095
        else:
            center=32768.0; scale=32768.0
            for w in current_words:
                I.append((w & 0xFFFF)-center)
                Q.append(((w>>16)&0xFFFF)-center)
            max_val = 65535
        c = np.array([(I[i]/scale) + 1j*(Q[i]/scale) for i in range(len(current_words))], dtype=np.complex128)
        c_corr = c - base_alpha * np.conjugate(c)
        # Reconstruct approximate words for next iteration
        I_corr = (np.real(c_corr)*scale + center).clip(0, max_val).astype(int)
        Q_corr = (np.imag(c_corr)*scale + center).clip(0, max_val).astype(int)
        new_words = []
        if map_mode=='dual12':
            for i_val, q_val in zip(I_corr, Q_corr):
                new_words.append((i_val & 0x0FFF) | ((q_val & 0x0FFF) << 16))
        else:
            for i_val, q_val in zip(I_corr, Q_corr):
                new_words.append((i_val & 0xFFFF) | ((q_val & 0xFFFF) << 16))
        current_words = new_words
    return base_alpha

def spectral_phase_select(buf, fs, analyze_words):
    """Evaluate all 4 byte phases via FFT tone/image ratio. Return best phase or None.
    Placed top-level so it's available for --spectral-align-test before main defines inner versions."""
    try:
        import numpy as np, math
    except Exception:
        return None
    if len(buf) < 2048:
        return None
    max_words = 4096
    best = None
    for phase in range(4):
        usable = len(buf) - phase
        if usable < 4:
            continue
        count = min(max_words, usable//4)
        words = []
        for i in range(count):
            off = phase + i*4
            b0 = buf[off]; b1 = buf[off+1]; b2 = buf[off+2]; b3 = buf[off+3]
            w = b0 | (b1<<8) | (b2<<16) | (b3<<24)
            words.append(w)
        I=[]; Q=[]; center=2048.0; scale=2048.0
        for w in words:
            I.append((w & 0x0FFF) - center)
            Q.append(((w>>16) & 0x0FFF) - center)
        c = np.array([(I[i]/scale) + 1j*(Q[i]/scale) for i in range(len(words))], dtype=np.complex128)
        N = c.shape[0]
        if N < 512:
            continue
        win = 0.5 - 0.5*np.cos(2*np.pi*np.arange(N)/(N-1))
        spec = np.fft.fft(c*win)
        mag2 = np.abs(spec)**2
        start_bin=1; end_bin=(N//2)-1
        main_bin = np.argmax(mag2[start_bin:end_bin+1]) + start_bin
        image_bin = (N - main_bin) % N
        main_pow = mag2[main_bin]; image_pow = mag2[image_bin]
        ratio = main_pow/(image_pow+1e-12)
        band = mag2[max(main_bin-1,0):min(main_bin+2,N)]
        conc = band.sum()/(mag2[:N//2].sum()+1e-12)
        score = conc * (ratio if ratio>1 else 0.1*ratio)
        if best is None or score > best[1]:
            best = (phase, score, conc, ratio, main_bin)
    if best:
        print(f"[spectral-align] phase={best[0]} score={best[1]:.3f} conc={best[2]:.3f} main/image ratio={best[3]:.1f} mainBin={best[4]}")
        return best[0]
    return None


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
    ap.add_argument('--fast-align', action='store_true', help='Deterministic initial alignment: choose phase with highest zero padding nibble ratio (skip spectral/heuristic).')
    ap.add_argument('--lock-phase-after-align', action='store_true', help='Disable auto-realign after first alignment (keep chosen phase fixed).')
    ap.add_argument('--expect-sync', metavar='HEX[,HEX...]', help='32-bit sync word or comma-separated multi-word frame expected at start (each hex like A55A5A7F). If found, alignment is forced to that byte boundary and the frame is discarded.')
    ap.add_argument('--sync-interval', type=int, default=0, help='Optional: expect sync word every N words (0=ignore). Host will warn if missing for >2*interval.')
    ap.add_argument('--realign-min-zero', type=float, default=0.80, help='If current phase zero-high ratio drops below this, consider realignment (default 0.80)')
    ap.add_argument('--realign-confirm', type=int, default=3, help='Number of consecutive windows a non-zero phase must dominate before shifting (hysteresis, default 3)')
    ap.add_argument('--realign-min-best-zero', type=float, default=0.90, help='Require best candidate zero-high ratio >= this before shifting (default 0.90)')
    ap.add_argument('--no-initial-align', action='store_true', help='Do not perform initial phase sniff before streaming (default: perform)')
    ap.add_argument('--initial-align-samples', type=int, default=256, help='Word count collected for initial alignment scoring (default 256)')
    ap.add_argument('--log-phase', type=int, metavar='N', help='Every N samples, log current alignment scores (diagnostic)')
    ap.add_argument('--force-phase', type=int, choices=[0,1,2,3], help='Force an initial byte phase (0..3) instead of auto detection')
    ap.add_argument('--prefer-phase', type=int, choices=[0,1,2,3], help='Preferred phase if scores within margin (tie-break)')
    ap.add_argument('--prefer-margin', type=float, default=0.01, help='Score difference threshold to allow choosing preferred phase (default 0.01)')
    ap.add_argument('--bias-phase0', action='store_true', help='Bias toward phase0 when zero_both_ratio ties (default ON unless prefer-phase specified)')
    ap.add_argument('--spectral-align', action='store_true', help='Use FFT-based tone concentration and image ratio to select initial byte phase (overrides heuristic metrics)')
    ap.add_argument('--spectral-align-test', action='store_true', help='Run synthetic tone buffer test for spectral alignment and exit')
    ap.add_argument('--exit-after-analyze', action='store_true', help='Perform analysis block then continue streaming unless --force-exit-after-analyze also given')
    ap.add_argument('--force-exit-after-analyze', action='store_true', help='With --exit-after-analyze, actually exit after analysis (legacy behavior)')
    ap.add_argument('--analyze-before-fifo', action='store_true', help='Perform --analyze / --dump-words before opening FIFO (avoids waiting for reader)')
    ap.add_argument('--measure-irr', type=int, metavar='N', help='Capture N words and estimate Image Rejection Ratio (IRR) using FFT (requires tone present)')
    ap.add_argument('--rate', type=float, default=DEFAULT_RATE, help='Sample rate used for IRR frequency mapping (Hz)')
    ap.add_argument('--irr-window', choices=['hann','rect'], default='hann', help='Window for IRR FFT (default hann)')
    ap.add_argument('--irr-min-hz', type=float, default=10.0, help='Ignore bins below this frequency when choosing main tone (default 10 Hz)')
    ap.add_argument('--irr-max-hz', type=float, default=None, help='Optional upper limit for search (Hz)')
    ap.add_argument('--irr-bin-margin', type=int, default=1, help='Bins around peak to integrate for amplitude (default 1)')
    ap.add_argument('--tone-hz', type=float, help='Expected primary tone frequency (Hz). Constrains IRR peak search to +/- tone-window-hz.')
    ap.add_argument('--tone-window-hz', type=float, default=500.0, help='Half-width (Hz) around --tone-hz to search for main bin (default 500 Hz)')
    ap.add_argument('--probe-layout', type=int, metavar='N', help='Capture N words and display multiple interpretations (endianness, lane positions)')
    ap.add_argument('--swap-lanes', action='store_true', help='Swap interpreted I/Q lanes (debug)')
    ap.add_argument('--big-endian-words', action='store_true', help='Interpret each 4-byte group as big-endian 32-bit before extraction (debug)')
    ap.add_argument('--invert-q', action='store_true', help='Invert Q channel polarity (around DC center) before output/analysis')
    ap.add_argument('--swap-iq-output', action='store_true', help='Swap I and Q order in FIFO output (use if downstream expects Q,I)')
    ap.add_argument('--dc-remove', action='store_true', help='Continuously remove running DC mean from I and Q before output (cf32/s8 modes)')
    ap.add_argument('--dc-alpha', type=float, default=0.001, help='IIR smoothing factor for DC removal (default 0.001)')
    ap.add_argument('--image-cancel', action='store_true', help='Apply single-coefficient image rejection (s = s - alpha*conjugate(s)) in cf32 output')
    ap.add_argument('--image-cancel-alpha', type=float, help='Optional explicit complex alpha (re,im) via two floats; if omitted auto-estimate from initial analyze block', nargs=2)
    ap.add_argument('--image-cancel-iter', type=int, metavar='K', help='Iteratively refine image cancel alpha over K iterations before streaming (uses analyze block if available)')
    ap.add_argument('--periodic-sync-block', action='store_true', help='Enable detection of periodic 64-byte sync blocks in stream and discard them.')
    ap.add_argument('--periodic-sync-block-interval', type=float, default=2.0, help='Expected minimum interval (seconds) between sync blocks (diagnostic warnings if blocks missing/excess).')
    ap.add_argument('--post-cancel-irr', action='store_true', help='After estimating alpha, apply cancellation to analyze block and report improved IRR')
    ap.add_argument('--auto-image-cancel', action='store_true', help='Automatically estimate/apply image cancellation (equivalent to --image-cancel with auto alpha)')
    ap.add_argument('--irr-after-cancel-stream', action='store_true', help='Measure IRR again after image cancellation begins during streaming')
    ap.add_argument('--alpha-neighbors', type=int, default=0, help='Integrate +/- this many neighbor bins for alpha estimation stability')
    ap.add_argument('--dump-first-hex', type=int, metavar='N', help='Dump first N raw serial bytes in hex (before alignment) and keep streaming')
    ap.add_argument('--periodic-alignment-check', type=int, metavar='SAMPLES', help='Every SAMPLES complex samples perform a 256-byte phase score check (debug)')
    ap.add_argument('--strip-sync-frame', action='store_true', help='Strip 16-byte periodic sync frame of 4 fixed words (0x7F5AA55A,0xA55A7F5A,0x13579BDF,0xFDB97531) inserted every 64 USB packets by firmware.')
    args = ap.parse_args()

    if args.spectral_align_test:
        try:
            import numpy as np, math
        except Exception:
            print('numpy required for --spectral-align-test')
            return
        fs = args.rate
        N_words = 4096
        tone_f = fs/32.0
        amp = 600
        mid = 2048
        buf = bytearray()
        for n in range(N_words):
            ang = 2*math.pi*tone_f*n/fs
            i_raw = int(mid + amp*math.cos(ang)) & 0x0FFF
            q_raw = int(mid + amp*math.sin(ang)) & 0x0FFF
            w = (q_raw<<16) | i_raw
            buf.append(w & 0xFF); buf.append((w>>8)&0xFF); buf.append((w>>16)&0xFF); buf.append((w>>24)&0xFF)
        print(f'[spectral-align-test] Generated {N_words} words tone_f={tone_f:.1f} Hz')
        sel_correct = spectral_phase_select(buf, fs, N_words)
        print(f'[spectral-align-test] Selection correct-buffer -> phase {sel_correct}')
        junk_prefixed = bytearray([0xAA]) + buf
        sel_misaligned = spectral_phase_select(junk_prefixed, fs, N_words)
        print(f'[spectral-align-test] Selection 1-byte-prefixed -> phase {sel_misaligned} (expect 1)')
        junk_prefixed3 = bytearray([0x11,0x22,0x33]) + buf
        sel_misaligned3 = spectral_phase_select(junk_prefixed3, fs, N_words)
        print(f'[spectral-align-test] Selection 3-byte-prefixed -> phase {sel_misaligned3} (expect 3)')
        return
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
        if args.measure_irr and args.measure_irr > args.analyze:
            print(f"[warn] --measure-irr ({args.measure_irr}) exceeds --analyze ({args.analyze}) in pre-analyze mode; only first {args.analyze} words available. Increase --analyze to >= --measure-irr for full IRR window.")
        while len(buf) < target_bytes:
            chunk = ser.read(args.chunk)
            if '_prefetched_initial_bytes' in globals():
                chunk = globals().pop('_prefetched_initial_bytes') + chunk
            if not chunk:
                continue
            buf.extend(chunk)
        # Apply spectral alignment if requested before building words
        phase_shift = 0
        if args.spectral_align:
            sel = spectral_phase_select(buf[:target_bytes], args.rate, args.analyze)
            if sel is not None:
                phase_shift = sel
                if phase_shift:
                    print(f"[pre-analyze] (spectral) Discarding {phase_shift} leading byte(s) prior to word decode.")
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
            if args.big_endian_words:
                w = (aligned[off] << 24) | (aligned[off+1] << 16) | (aligned[off+2] << 8) | aligned[off+3]
            else:
                w = aligned[off] | (aligned[off+1] << 8) | (aligned[off+2] << 16) | (aligned[off+3] << 24)
            words.append(w)
        if args.dump_words:
            print("RAW WORDS:")
            for i,w in enumerate(words):
                print(f" {i:04d}: 0x{w:08X}")
        analyze_samples(words)
        if args.measure_irr:
            print(f"[irr] Measuring IRR on first {args.measure_irr} words (pre-analyze phase)...")
            irr_words = words[:args.measure_irr]
            irr_initial = measure_irr_from_words(irr_words, args.rate, map_mode=args.map, window=args.irr_window,
                                                 min_hz=args.irr_min_hz, max_hz=args.irr_max_hz, bin_margin=args.irr_bin_margin,
                                                 invert_q=args.invert_q)
            if args.post_cancel_irr and irr_initial:
                # Estimate alpha then apply to a reconstructed complex block and re-measure
                est_alpha = estimate_image_alpha(irr_words, fs=args.rate, map_mode=args.map)
                if est_alpha is not None:
                    # Rebuild complex samples
                    I=[]; Q=[]; center = (2048 if args.map=='dual12' else 32768); scale=center
                    for w in irr_words:
                        if args.map=='dual12':
                            I.append((w & 0x0FFF) - center)
                            Q.append(((w>>16)&0x0FFF) - center)
                        else:
                            I.append((w & 0xFFFF) - center)
                            Q.append(((w>>16)&0xFFFF) - center)
                    c = np.array([(I[i]/scale)+1j*(Q[i]/scale) for i in range(len(irr_words))], dtype=np.complex128)
                    c_corr = c - est_alpha * np.conjugate(c)
                    # Re-pack into pseudo words for IRR measurement reuse
                    rec_words=[]
                    if args.map=='dual12':
                        for s in c_corr:
                            i_val = int((s.real*scale + center)) & 0x0FFF
                            q_val = int((s.imag*scale + center)) & 0x0FFF
                            rec_words.append(i_val | (q_val<<16))
                    else:
                        for s in c_corr:
                            i_val = int((s.real*scale + center)) & 0xFFFF
                            q_val = int((s.imag*scale + center)) & 0xFFFF
                            rec_words.append(i_val | (q_val<<16))
                    irr_post = measure_irr_from_words(rec_words, args.rate, map_mode=args.map, window=args.irr_window,
                                                      min_hz=args.irr_min_hz, max_hz=args.irr_max_hz, bin_margin=args.irr_bin_margin,
                                                      invert_q=args.invert_q)
                    if irr_post and irr_post.get('irr_db') is not None:
                        print(f"[irr-post-cancel] alpha={est_alpha.real:.3e}+{est_alpha.imag:.3e}j improved IRR from {irr_initial.get('irr_db',0):.1f} dB to {irr_post.get('irr_db',0):.1f} dB")
                else:
                    print('[irr-post-cancel] Failed to estimate alpha (insufficient tone).')
            # Fallback: if IRR printed negative (image > main) try reinterpret in phase0 framing by dropping initial shift if taken
            # Simple heuristic: if we discarded 1-3 bytes earlier (initial-align shift) and IRR < 0, rebuild words from original buffer with no shift and re-run.
            # Placeholder for potential realignment based on initial-align logs; removed getvalue() misuse.
        if args.probe_layout:
            probe_layout(words[:args.probe_layout], swap_lanes=args.swap_lanes, big_endian=args.big_endian_words)
        if args.exit_after_analyze and args.force_exit_after_analyze:
            print("[pre-analyze] Force-exit requested after analysis.")
            ser.close()
            return
        # continue streaming -> fall through to FIFO setup

    fifo_path = args.fifo
    ensure_fifo(fifo_path)
    print(f"FIFO ready: {fifo_path} (waiting for reader when opened) format={args.format}")

    # Open FIFO for write only; on macOS need reader first or open blocks.
    fifo_fd = None
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
    irr_words = []
    # State for DC removal and image cancellation
    dc_i = 0.0
    dc_q = 0.0
    img_alpha = None  # complex cancellation coefficient
    img_alpha_fixed = False
    raw_accum = bytearray() if args.validate_alignment else None
    alignment_done = False
    realign_buffer = bytearray()
    last_realign_at_samples = 0
    initial_alignment_done = args.no_initial_align
    initial_align_buffer = bytearray()
    candidate_phase = None
    candidate_count = 0
    # --- Periodic 16-byte sync frame removal state (firmware main loop inserts these every 64 USB IN packets) ---
    SYNC_FRAME_WORDS = (0x7F5AA55A,0xA55A7F5A,0x13579BDF,0xFDB97531)
    SYNC_FRAME_LEN = 16
    SYNC_FRAME_BYTES = b''.join(struct.pack('<I', w) for w in SYNC_FRAME_WORDS)
    sync_scan_tail = b''  # retain last 15 bytes between reads to catch boundary-spanning frames
    sync_frames_removed = 0

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
                metrics.append({'phase':phase,'words':0,'score':-1,'zero_hi_b1':0,'zero_hi_b3':0,'zero_both_ratio':0,'span':0,'out_of_range':1.0,'pad_zero_ratio':0.0})
                continue
            words = min(words_target, usable//4)
            if words == 0:
                metrics.append({'phase':phase,'words':0,'score':-1,'zero_hi_b1':0,'zero_hi_b3':0,'zero_both_ratio':0,'span':0,'out_of_range':1.0,'pad_zero_ratio':0.0})
                continue
            minv = 0xFFFF; maxv = 0
            z1 = z2 = both = oor = 0
            pad_zero = 0
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
                # Padding nibble (bits12..15 of each lane) should be zero for correct dual12 alignment
                # Check by inspecting high nibble of the 16-bit lane's high byte (b1>>4 and b3>>4) staying 0x0 or occasionally 0x1 when near boundary
                if ((b1 >> 4) == 0) and ((b3 >> 4) == 0):
                    pad_zero += 1
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
            pad_zero_ratio = pad_zero/words
            # scoring: favor high both_ratio, low oor, reasonable span (> small threshold) to ensure not flat
            span_norm = min(span/4095.0, 1.0)
            # Increase weight for pad_zero_ratio to discriminate correct framing (expected near 1.0 for phase0, low for misalignment)
            score = both_ratio*0.45 + pad_zero_ratio*0.35 + span_norm*0.15 - oor_frac*0.25 + (zero_hi_b1+zero_hi_b3)*0.05
            metrics.append({'phase':phase,'words':words,'score':score,'zero_hi_b1':zero_hi_b1,'zero_hi_b3':zero_hi_b3,'zero_both_ratio':both_ratio,'span':span,'out_of_range':oor_frac,'pad_zero_ratio':pad_zero_ratio})
        return metrics



    try:
        while True:
            chunk = ser.read(args.chunk)
            if '_prefetched_initial_bytes' in globals():
                chunk = globals().pop('_prefetched_initial_bytes') + chunk
            if not chunk:
                continue
            # Lightweight in-stream removal of 16-byte sync frame (only after initial alignment so we know phase0 framing)
            if args.strip_sync_frame and initial_alignment_done:
                scan_buf = sync_scan_tail + chunk
                # We only need to scan up to len-15 (a full frame is 16 bytes)
                limit = len(scan_buf) - (SYNC_FRAME_LEN - 1)
                pos = 0
                cleaned = bytearray()
                while pos < limit:
                    if scan_buf[pos:pos+SYNC_FRAME_LEN] == SYNC_FRAME_BYTES:
                        sync_frames_removed += 1
                        if sync_frames_removed <= 3:  # log first few for confirmation
                            print(f"[sync-frame] Removed 16-byte frame #{sync_frames_removed} (running total)")
                        pos += SYNC_FRAME_LEN
                        continue
                    cleaned.append(scan_buf[pos])
                    pos += 1
                # Append remaining tail bytes (could be <16; keep for next iteration scan)
                sync_scan_tail = scan_buf[pos:]
                chunk = bytes(cleaned)
            if args.skip_stat:
                printable = sum(32 <= b < 127 for b in chunk)
                if printable/len(chunk) > ASCII_SKIP_THRESHOLD and b'STAT' in chunk:
                    continue
            if raw_capture_fh:
                raw_capture_fh.write(chunk)

            # Periodic sync block detection (64 bytes) after initial alignment complete
            if args.periodic_sync_block and initial_alignment_done and len(chunk) >= 64:
                # Block signature pattern (first 4 words) we expect: 0x7F5AA55A,0xA55A7F5A,0x13579BDF,0xFDB97531
                # plus additional structure and xor checksum on last word; perform sliding scan within chunk
                SYNC_A=0x7F5AA55A; SYNC_B=0xA55A7F5A; SYNC_C=0x13579BDF; SYNC_D=0xFDB97531
                found_sync_offsets=[]
                buf=chunk
                for off in range(0, len(buf)-63):
                    w0 = buf[off] | (buf[off+1]<<8) | (buf[off+2]<<16) | (buf[off+3]<<24)
                    if w0 != SYNC_A: continue
                    w1 = buf[off+4] | (buf[off+5]<<8) | (buf[off+6]<<16) | (buf[off+7]<<24)
                    w2 = buf[off+8] | (buf[off+9]<<8) | (buf[off+10]<<16) | (buf[off+11]<<24)
                    w3 = buf[off+12] | (buf[off+13]<<8) | (buf[off+14]<<16) | (buf[off+15]<<24)
                    if (w1,w2,w3) != (SYNC_B,SYNC_C,SYNC_D): continue
                    # Compute xor checksum of first 15 words and compare with word15
                    words=[]
                    for i in range(16):
                        base=off+i*4
                        wi = buf[base] | (buf[base+1]<<8) | (buf[base+2]<<16) | (buf[base+3]<<24)
                        words.append(wi)
                    chk=0
                    for wi in words[:15]: chk ^= wi
                    if chk == words[15]:
                        found_sync_offsets.append(off)
                if found_sync_offsets:
                    now_time = time.time()
                    if not hasattr(args,'_last_sync_block_time'): args._last_sync_block_time = 0
                    for off in found_sync_offsets:
                        # Remove the block from processing: we slice chunk around it
                        block = chunk[off:off+64]
                        chunk = chunk[:off] + chunk[off+64:]
                        interval = now_time - getattr(args,'_last_sync_block_time',0)
                        print(f"[psync] Sync block detected at offset {off} within chunk (interval {interval:.3f}s)")
                        args._last_sync_block_time = now_time
                        # Warn if interval too short or long relative to expected
                        if interval > 0 and args.periodic_sync_block_interval > 0:
                            if interval < args.periodic_sync_block_interval*0.5:
                                print(f"[psync] Warning: sync block interval {interval:.2f}s < 50% expected {args.periodic_sync_block_interval}s")
                            elif interval > args.periodic_sync_block_interval*2.5:
                                print(f"[psync] Warning: sync block interval {interval:.2f}s > 250% expected {args.periodic_sync_block_interval}s")

            # Before initial alignment completes, accumulate only in initial_align_buffer (not in partial)
            if not initial_alignment_done and args.map=='dual12':
                initial_align_buffer.extend(chunk)
                # Sync word detection (preempts all other alignment strategies)
                if args.expect_sync:
                    # Parse either single hex or comma-separated list for frame sequence
                    sync_parts = [p.strip() for p in args.expect_sync.split(',') if p.strip()]
                    sync_words = []
                    for part in sync_parts:
                        try:
                            sync_words.append(int(part, 16) & 0xFFFFFFFF)
                        except ValueError:
                            print(f"[sync] Invalid hex element in --expect-sync: {part}")
                            sync_words = []
                            break
                    if sync_words:
                        buf = initial_align_buffer
                        found_phase = None; found_index = None
                        scan_limit = min(len(buf), 8192*4)
                        frame_len_bytes = len(sync_words) * 4
                        for phase in range(4):
                            usable = scan_limit - phase
                            if usable < frame_len_bytes: continue
                            # Scan all offsets for first word match then verify subsequent words
                            for off in range(phase, scan_limit - frame_len_bytes + 1):
                                w0 = buf[off] | (buf[off+1]<<8) | (buf[off+2]<<16) | (buf[off+3]<<24)
                                if w0 == sync_words[0]:
                                    match = True
                                    # verify rest
                                    for wi in range(1, len(sync_words)):
                                        o2 = off + wi*4
                                        w = buf[o2] | (buf[o2+1]<<8) | (buf[o2+2]<<16) | (buf[o2+3]<<24)
                                        if w != sync_words[wi]:
                                            match = False
                                            break
                                    if match:
                                        found_phase = (off - phase) % 4
                                        found_index = off
                                        break
                            if found_phase is not None:
                                break
                        if found_phase is not None:
                            shift = found_phase
                            start = found_index + len(sync_words)*4
                            # Discard sync frame and any leading bytes prior to frame to align
                            partial = buf[start:]
                            if len(partial) % 4:
                                partial = partial[:len(partial) - (len(partial)%4)]
                            initial_alignment_done = True
                            realign_buffer.extend(initial_align_buffer)
                            frame_hex = ','.join(f"0x{w:08X}" for w in sync_words)
                            print(f"[sync] Found sync frame [{frame_hex}] at byte {found_index} phase={found_phase} (scan {scan_limit} bytes). Alignment locked.")
                            if args.lock_phase_after_align:
                                args.auto_realign = False
                                print("[sync] Phase locked (auto-realign disabled).")
                            continue
                        else:
                            if len(buf) >= scan_limit and not getattr(args, '_sync_notified', False):
                                if len(sync_words) == 1:
                                    print(f"[sync] Sync 0x{sync_words[0]:08X} not found in first {scan_limit} bytes; falling back to normal alignment.")
                                else:
                                    frame_hex = ','.join(f"0x{w:08X}" for w in sync_words)
                                    print(f"[sync] Sync frame [{frame_hex}] not found in first {scan_limit} bytes; falling back to normal alignment.")
                                args._sync_notified = True
                # Fast deterministic alignment option
                if args.fast_align and len(initial_align_buffer) >= (args.initial_align_samples*4):
                    needed = args.initial_align_samples*4
                    buf = initial_align_buffer[:needed]
                    # Score phases by padding nibble zeros (bits12..15 and 28..31 expected zero) -> check high nibble of byte1 and byte3 of each 4-byte word
                    phase_scores = []
                    for phase in range(4):
                        usable = len(buf) - phase
                        if usable < 4:
                            phase_scores.append((phase, -1, 0))
                            continue
                        words = usable // 4
                        pad_zero = 0
                        for i in range(words):
                            off = phase + i*4
                            b1 = buf[off+1]; b3 = buf[off+3]
                            if (b1 >> 4) == 0: pad_zero += 1
                            if (b3 >> 4) == 0: pad_zero += 1
                        ratio = pad_zero/(words*2) if words else 0
                        phase_scores.append((phase, ratio, words))
                    best = max(phase_scores, key=lambda t: t[1])
                    shift = best[0]
                    partial = initial_align_buffer[shift:]
                    if len(partial) % 4:
                        partial = partial[:len(partial) - (len(partial) % 4)]
                    print(f"[fast-align] Selected phase {shift} (padZeroRatio={best[1]*100:.2f}% words={best[2]})")
                    initial_alignment_done = True
                    realign_buffer.extend(initial_align_buffer)
                    if args.lock_phase_after_align:
                        args.auto_realign = False
                        print("[fast-align] Phase locked (auto-realign disabled).")
                    continue
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
                        if args.spectral_align:
                            sel = spectral_phase_select(initial_align_buffer[:needed], args.rate, args.analyze)
                            if sel is not None:
                                shift = sel
                                if shift != 0:
                                    partial = initial_align_buffer[shift:]
                                    print(f"[initial-align] (spectral) Selected phase {shift} (discarded {shift} byte(s)).")
                                else:
                                    partial = initial_align_buffer
                                    print(f"[initial-align] (spectral) Selected phase 0.")
                                initial_alignment_done = True
                                realign_buffer.extend(initial_align_buffer)
                                # Optional immediate IRR measurement pre-cancellation
                                if args.measure_irr:
                                    raw_words_for_irr = []
                                    block = initial_align_buffer[shift:][:args.measure_irr*4]
                                    # Truncate to 4-byte boundary to avoid IndexError on last incomplete word
                                    blk_len = len(block) - (len(block) % 4)
                                    block = block[:blk_len]
                                    for off in range(0, min(len(block), args.measure_irr*4), 4):
                                        # Safety: ensure we have a full 4-byte word
                                        if off+3 >= len(block):
                                            break
                                        w = block[off] | (block[off+1]<<8) | (block[off+2]<<16) | (block[off+3]<<24)
                                        raw_words_for_irr.append(w)
                                    irr_pre = measure_irr_from_words(raw_words_for_irr, args.rate, map_mode=args.map, window=args.irr_window,
                                                                 min_hz=args.irr_min_hz, max_hz=args.irr_max_hz, bin_margin=args.irr_bin_margin,
                                                                 invert_q=args.invert_q)
                                    if irr_pre and irr_pre.get('irr_db',0) < 0:
                                        print(f"[irr-fallback] Negative IRR detected ({irr_pre['irr_db']:.1f} dB). Re-evaluating phases spectrally.")
                                        # Try all phase shifts and pick best IRR
                                        best_phase=None; best_irr=-1e9
                                        for p in range(4):
                                            words_tmp=[]
                                            blk = initial_align_buffer[p:][:args.measure_irr*4]
                                            for off in range(0, min(len(blk), args.measure_irr*4), 4):
                                                w = blk[off] | (blk[off+1]<<8) | (blk[off+2]<<16) | (blk[off+3]<<24)
                                                words_tmp.append(w)
                                            irr_tmp = measure_irr_from_words(words_tmp, args.rate, map_mode=args.map, window=args.irr_window,
                                                                            min_hz=args.irr_min_hz, max_hz=args.irr_max_hz, bin_margin=args.irr_bin_margin,
                                                                            invert_q=args.invert_q)
                                            if irr_tmp and irr_tmp.get('irr_db', -1e9) > best_irr:
                                                best_irr = irr_tmp['irr_db']; best_phase = p
                                        if best_phase is not None and best_phase != shift:
                                            print(f"[irr-fallback] Switching phase {shift} -> {best_phase} (IRR improved to {best_irr:.1f} dB)")
                                            shift = best_phase
                                            partial = initial_align_buffer[shift:]
                                # End IRR fallback block
                                continue
                        metrics = compute_phase_metrics(initial_align_buffer[:needed], args.initial_align_samples)
                        # Re-score emphasizing zero_both_ratio dominance
                        # Already computed score includes both_ratio heavy weight; we still explicitly pick by both_ratio then score
                        # Determine best candidate(s)
                        # Primary key: zero_both_ratio, secondary: score
                        sorted_m = sorted(metrics, key=lambda m: (m['zero_both_ratio'], m['score'], m['span']), reverse=True)
                        best = sorted_m[0]
                        # Phase0 bias: if enabled (implicit unless prefer-phase set) and multiple candidates share zero_both_ratio within 0.2% & score within prefer-margin, choose phase0 if present and its span not worse.
                        if (args.bias_phase0 or (args.prefer_phase is None and not args.force_phase and not args.no_initial_align)):
                            candidates = [m for m in metrics if abs(m['zero_both_ratio']-best['zero_both_ratio']) <= 0.002 and abs(m['score']-best['score']) <= args.prefer_margin]
                            p0 = next((m for m in candidates if m['phase']==0), None)
                            if p0 and p0['phase'] != best['phase']:
                                # Require span within 5% of best span or greater
                                if p0['span'] >= 0.95*best['span']:
                                    print(f"[initial-align] Phase0 bias applied (tie) selecting phase0 over {best['phase']} span0={p0['span']} spanBest={best['span']}")
                                    best = p0
                        shift = best['phase']
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
                            # Safeguard: if phase0 pad_zero_ratio very high (>=95%) but not selected, reconsider
                            p0m = next((m for m in metrics if m['phase']==0), None)
                            if p0m and p0m['pad_zero_ratio'] >= 0.95 and best['pad_zero_ratio'] < 0.95:
                                print(f"[initial-align] Override: phase0 pad_zero_ratio={p0m['pad_zero_ratio']*100:.1f}% dominates -> choosing phase0 instead of phase {shift}")
                                shift = 0; best = p0m
                            partial = initial_align_buffer[shift:]
                            # Trim to full words boundary (4 bytes each) to avoid incomplete trailing bytes
                            if len(partial) % 4:
                                partial = partial[:len(partial) - (len(partial) % 4)]
                            print(f"[initial-align] Selected phase {shift} (discarded {shift} byte(s)). both={best['zero_both_ratio']*100:.1f}% padZ={best['pad_zero_ratio']*100:.1f}% score={best['score']:.3f} span={best['span']}")
                        else:
                            partial = initial_align_buffer
                            if len(partial) % 4:
                                partial = partial[:len(partial) - (len(partial) % 4)]
                            print(f"[initial-align] Selected phase 0. both={best['zero_both_ratio']*100:.1f}% padZ={best['pad_zero_ratio']*100:.1f}% score={best['score']:.3f} span={best['span']}")
                        tbl = ' '.join(f"p{m['phase']}:{m['zero_both_ratio']*100:.0f}% padZ={m['pad_zero_ratio']*100:.0f}% span={m['span']}/score={m['score']:.3f}" for m in metrics)
                        print(f"[initial-align-metrics] {tbl}")
                        realign_buffer.extend(initial_align_buffer)
                        initial_alignment_done = True
                        # Pre-cancellation IRR report (heuristic path)
                        if args.measure_irr:
                            raw_words_for_irr = []
                            block = partial[:args.measure_irr*4]
                            blk_len = len(block) - (len(block) % 4)
                            block = block[:blk_len]
                            for off in range(0, min(len(block), args.measure_irr*4), 4):
                                if off+3 >= len(block):
                                    break
                                w = block[off] | (block[off+1]<<8) | (block[off+2]<<16) | (block[off+3]<<24)
                                raw_words_for_irr.append(w)
                            irr_pre = measure_irr_from_words(raw_words_for_irr, args.rate, map_mode=args.map, window=args.irr_window,
                                                             min_hz=args.irr_min_hz, max_hz=args.irr_max_hz, bin_margin=args.irr_bin_margin,
                                                             invert_q=args.invert_q)
                            if irr_pre and irr_pre.get('irr_db',0) < 0 and args.spectral_align:
                                print(f"[irr-fallback] Negative IRR after heuristic alignment; attempting spectral phase re-select.")
                                sel2 = spectral_phase_select(initial_align_buffer[:needed], args.rate, analyze_words)
                                if sel2 is not None and sel2 != shift:
                                    print(f"[irr-fallback] Spectral override shift {shift}->{sel2}")
                                    shift = sel2
                                    partial = initial_align_buffer[shift:]
                                    realign_buffer.clear(); realign_buffer.extend(initial_align_buffer)
                # Until alignment done, skip rest of loop
                if not initial_alignment_done:
                    continue
            else:
                # Alignment already done or not applicable -> append chunk to partial
                partial += chunk

            # (Initial alignment logic moved earlier; this block intentionally removed)

            # Disable auto-realign if user locked phase after initial alignment
            if getattr(args, 'lock_phase_after_align', False) and initial_alignment_done:
                args.auto_realign = False
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
                        if args.big_endian_words:
                            w = (block[off] << 24) | (block[off+1] << 16) | (block[off+2] << 8) | block[off+3]
                        else:
                            w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                        analyze_words.append(w)
                if len(analyze_words) >= args.analyze:
                    if args.dump_words:
                        print("RAW WORDS:")
                        for i,w in enumerate(analyze_words):
                            print(f" {i:04d}: 0x{w:08X}")
                    analyze_samples(analyze_words)
                    if args.probe_layout:
                        probe_layout(analyze_words[:args.probe_layout], swap_lanes=args.swap_lanes, big_endian=args.big_endian_words)
                    analyzed = True
                    if args.exit_after_analyze and args.force_exit_after_analyze:
                        print('[stream-analyze] Force-exit requested after analysis inside streaming path. Exiting.')
                        break
            # Tail capture path (last N words across entire session)
            if tail_words is not None:
                for off in range(0, len(block), 4):
                    w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                    tail_words.append(w)
            if args.format in ('u8','s8'):
                outbuf = bytearray()
                decode_block(block, args.format, outbuf, zero_i=args.zero_i, zero_q=args.zero_q, map_mode=args.map, invert_q=args.invert_q)
                if args.swap_iq_output:
                    # Swap bytes in-place: (I,Q) pairs become (Q,I)
                    for i in range(0, len(outbuf)-1, 2):
                        outbuf[i], outbuf[i+1] = outbuf[i+1], outbuf[i]
                total_samples += len(outbuf)//2
            elif args.format == 'raw16':
                outbuf = bytearray()
                decode_block(block, 'raw16', outbuf, zero_i=args.zero_i, zero_q=args.zero_q, map_mode=args.map, invert_q=args.invert_q)
                if args.swap_iq_output:
                    # raw16 layout: I_lo,I_hi,Q_lo,Q_hi per sample; swap 4-byte groups to Q,I
                    swapped = bytearray()
                    for i in range(0, len(outbuf), 4):
                        if i+3 >= len(outbuf): break
                        q_lo = outbuf[i+2]; q_hi = outbuf[i+3]; i_lo = outbuf[i]; i_hi = outbuf[i+1]
                        swapped.append(q_lo); swapped.append(q_hi); swapped.append(i_lo); swapped.append(i_hi)
                    outbuf = swapped
                total_samples += len(outbuf)//4
            else:  # cf32
                pieces = []
                decode_block(block, 'cf32', pieces, zero_i=args.zero_i, zero_q=args.zero_q, map_mode=args.map, invert_q=args.invert_q)
                # Apply DC removal / image cancellation if requested
                # Enable implicit image cancel if --auto-image-cancel given
                if args.auto_image_cancel:
                    args.image_cancel = True
                if args.dc_remove or args.image_cancel:
                    new_pieces = []
                    # Auto-estimate alpha once using analyze_words if requested and not fixed
                    if args.image_cancel and img_alpha is None and not img_alpha_fixed:
                        if args.image_cancel_alpha:
                            re, im = args.image_cancel_alpha
                            img_alpha = complex(re, im)
                            img_alpha_fixed = True
                            print(f"[image-cancel] Using provided alpha={img_alpha}")
                        elif analyze_words:
                            # Estimate alpha from first analyze block: ratio of image/main bins
                            # Reconstruct complex samples from analyze_words for estimation
                            est_alpha = estimate_image_alpha(analyze_words, fs=args.rate, map_mode=args.map)
                            if est_alpha is not None:
                                img_alpha = est_alpha
                                print(f"[image-cancel] Estimated alpha={img_alpha.real:.3e}+{img_alpha.imag:.3e}j from initial block")
                                if args.image_cancel_iter:
                                    refined_alpha = refine_image_alpha(analyze_words, args.rate, args.map, iterations=args.image_cancel_iter)
                                    if refined_alpha != 0+0j:
                                        img_alpha = refined_alpha
                                        img_alpha_fixed = True
                                        print(f"[image-cancel-iter] Refined alpha over {args.image_cancel_iter} iterations -> {img_alpha.real:.3e}+{img_alpha.imag:.3e}j")
                    for raw_pair in pieces:
                        fi, fq = struct.unpack('<ff', raw_pair)
                        if args.dc_remove:
                            # IIR mean removal
                            dc_i = (1.0 - args.dc_alpha) * dc_i + args.dc_alpha * fi
                            dc_q = (1.0 - args.dc_alpha) * dc_q + args.dc_alpha * fq
                            fi -= dc_i
                            fq -= dc_q
                        if args.image_cancel and img_alpha is not None:
                            # s_corr = s - alpha*conj(s)
                            s = complex(fi, fq)
                            sc = s - img_alpha * s.conjugate()
                            fi = sc.real; fq = sc.imag
                        new_pieces.append(struct.pack('<ff', fi, fq))
                    outbuf = b''.join(new_pieces)
                else:
                    outbuf = b''.join(pieces)
                if args.swap_iq_output:
                    # cf32: sequence of 8-byte (float32 I, float32 Q). Swap to (Q,I)
                    swapped = bytearray()
                    for i in range(0, len(outbuf), 8):
                        if i+7 >= len(outbuf): break
                        fi = outbuf[i:i+4]; fq = outbuf[i+4:i+8]
                        swapped.extend(fq); swapped.extend(fi)
                    outbuf = bytes(swapped)
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
            # Periodic optional alignment check (debug only)
            if args.periodic_alignment_check:
                if total_samples and (total_samples % args.periodic_alignment_check)==0:
                    try:
                        validate_alignment(block[:256], 32)
                    except Exception as e:
                        print(f"[align-check-error] {e}")
            # Periodic progress every ~2 seconds
            if total_samples and (total_samples % (DEFAULT_RATE*2) == 0):
                elapsed = time.time() - start_time
                rate = total_samples/elapsed if elapsed>0 else 0.0
                print(f"Samples={total_samples}  OutBytes={total_bytes_out}  Rate={rate:.1f} sps ({rate/1000:.1f} ksps) fmt={args.format}")
            # IRR measurement capture (non-blocking); collect words until target then compute once
            if args.measure_irr and len(irr_words) < args.measure_irr:
                # Reconstruct raw words from block
                for off in range(0, len(block), 4):
                    if len(irr_words) >= args.measure_irr:
                        break
                    if args.big_endian_words:
                        w = (block[off] << 24) | (block[off+1] << 16) | (block[off+2] << 8) | block[off+3]
                    else:
                        w = block[off] | (block[off+1] << 8) | (block[off+2] << 16) | (block[off+3] << 24)
                    irr_words.append(w)
                if len(irr_words) >= args.measure_irr:
                    print(f"[irr] Measuring IRR after collecting {len(irr_words)} words during streaming...")
                    measure_irr_from_words(irr_words, args.rate, map_mode=args.map, window=args.irr_window,
                                           min_hz=args.irr_min_hz, max_hz=args.irr_max_hz, bin_margin=args.irr_bin_margin,
                                           invert_q=args.invert_q)
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
        if args.strip_sync_frame:
            print(f"[sync-frame] Total removed sync frames: {sync_frames_removed}")
        if tail_words is not None and len(tail_words):
            print(f"TAIL RAW WORDS (last {len(tail_words)}):")
            base_index = total_samples - len(tail_words)
            for i,w in enumerate(tail_words):
                print(f" {base_index + i:08d}: 0x{w:08X}")

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

def measure_irr_from_words(words, fs, *, map_mode='dual12', window='hann', min_hz=10.0, max_hz=None, bin_margin=1, invert_q=False, tone_hz=None, tone_window_hz=500.0):
    """Estimate image rejection ratio (IRR) from a block of raw packed words.

    Steps:
      1. Extract I,Q lanes per map_mode (dual12 or split16) and center them.
      2. Form complex sequence c = (I - center) + j*(Q - center) normalized to [-1,1] approx.
      3. Apply optional window (Hann) to reduce spectral leakage.
      4. Compute magnitude spectrum via FFT (numpy if available, else simple DFT for N<=4096).
      5. Find strongest positive-frequency bin (exclude DC and < min_hz).
      6. Identify image bin at mirrored negative frequency (index N-k).
      7. Integrate power across +/- bin_margin around each peak.
      8. IRR(dB) = 10*log10(main_power / image_power).
      9. Derive equivalent small-error magnitude sqrt(eps^2 + phi^2) = 2 / (10^(IRR/20)).

    Notes:
      - Requires a reasonably pure single tone; noisy wideband will produce misleading IRR.
      - Use --tone-hz to constrain search if you know the tone frequency.
      - Provide --rate to map bins to Hz accurately.
      - Image bin is mirrored negative frequency (N - main_bin).
    """

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

# Ensure script executes main() when run directly.
if __name__ == '__main__':
    main()
