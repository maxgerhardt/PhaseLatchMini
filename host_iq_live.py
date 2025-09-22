#!/usr/bin/env python3
"""
Live I/Q viewer for the STM32 ADC streaming build.

Assumptions:
  - Device enumerates as USB CDC (/dev/tty.usbmodem*)
  - Stream consists primarily of packed 32-bit little-endian words.
  - Each 32-bit word encodes I (lower 12 bits) and Q (next 12 bits) as:
        bits 11..0   : I sample (0..4095)
        bits 23..12  : Q sample (0..4095)
    (upper bits 31..24 ignored / may contain sequence or noise)
  - No textual STAT lines are mixed in; if present, textual lines are skipped.

Features:
  - Smooth, low-CPU terminal updating with recent samples.
  - Optional running min/max tracking per channel.
  - Simple ASCII sparkline (coarse) if enabled.
  - Graceful handling of partial frames and stray ASCII.

Usage examples:
  python host_iq_live.py              # auto-detect port, show I/Q stream
  python host_iq_live.py --port /dev/tty.usbmodem14101 --rate 500
  python host_iq_live.py --spark --window 64

Press Ctrl+C to exit.
"""
import argparse, glob, sys, time, math, os, re

try:
    import serial  # pyserial
except ImportError:
    print("ERROR: pyserial not installed: pip install pyserial", file=sys.stderr)
    sys.exit(2)

TEXT_LINE_RE = re.compile(rb'STAT |\r?\n')  # crude detector for text interruptions

ASCII_THRESHOLD = 0.92  # if >92% bytes in a block are printable, treat as text noise

SPARK_CHARS = "▁▂▃▄▅▆▇█"


def find_port(explicit=None):
    if explicit:
        return explicit
    ports = sorted(glob.glob('/dev/tty.usbmodem*'))
    if not ports:
        raise SystemExit("No /dev/tty.usbmodem* device found")
    return ports[0]


def spark(values):
    if not values:
        return ''
    lo = min(values); hi = max(values)
    if hi == lo:
        return SPARK_CHARS[0] * len(values)
    span = hi - lo
    out = []
    for v in values:
        idx = int((v - lo) / span * (len(SPARK_CHARS)-1) + 1e-9)
        if idx < 0: idx = 0
        if idx >= len(SPARK_CHARS): idx = len(SPARK_CHARS)-1
        out.append(SPARK_CHARS[idx])
    return ''.join(out)


def decode_words(buf):
    # Return list of (i,q) from a bytes-like object length multiple of 4
    out = []
    for off in range(0, len(buf), 4):
        w = buf[off] | (buf[off+1] << 8) | (buf[off+2] << 16) | (buf[off+3] << 24)
        i = w & 0x0FFF
        q = (w >> 12) & 0x0FFF
        out.append((i,q))
    return out


def main():
    ap = argparse.ArgumentParser(description='Live I/Q 12-bit sample viewer (CDC serial)')
    ap.add_argument('--port', help='Explicit serial port path')
    ap.add_argument('--baud', type=int, default=115200)  # ignored by USB but keeps API uniform
    ap.add_argument('--rate', type=float, default=20.0, help='UI refresh rate (Hz)')
    ap.add_argument('--window', type=int, default=32, help='Number of recent samples to display')
    ap.add_argument('--spark', action='store_true', help='Show ASCII sparkline per channel')
    ap.add_argument('--stats', action='store_true', help='Show running min/max/avg')
    ap.add_argument('--raw-dump', action='store_true', help='Also print raw hex words (debug)')
    ap.add_argument('--timeout', type=float, default=0.01, help='Serial read timeout seconds')
    args = ap.parse_args()

    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=args.timeout)

    recent_i = []
    recent_q = []
    min_i = 4095; max_i = 0; sum_i = 0; count_i = 0
    min_q = 4095; max_q = 0; sum_q = 0; count_q = 0

    next_ui = time.time()
    ui_interval = 1.0 / max(args.rate, 1e-3)

    partial = b''

    try:
        while True:
            chunk = ser.read(512)
            if chunk:
                # Filter out any ASCII STAT lines or stray text: skip if mostly printable
                printable = sum(32 <= b < 127 for b in chunk)
                if printable / len(chunk) > ASCII_THRESHOLD and b'STAT ' in chunk:
                    continue
                partial += chunk
                # Align to 4-byte boundary
                if len(partial) < 4:
                    continue
                cut = len(partial) - (len(partial) % 4)
                block = partial[:cut]
                partial = partial[cut:]
                words = decode_words(block)
                for i,q in words:
                    recent_i.append(i); recent_q.append(q)
                    min_i = min(min_i, i); max_i = max(max_i, i); sum_i += i; count_i += 1
                    min_q = min(min_q, q); max_q = max(max_q, q); sum_q += q; count_q += 1
                # Trim windows
                if len(recent_i) > args.window:
                    recent_i = recent_i[-args.window:]
                    recent_q = recent_q[-args.window:]
                if args.raw_dump:
                    for (i,q) in words:
                        print(f"{i:03X}:{q:03X}")
            now = time.time()
            if now >= next_ui:
                next_ui = now + ui_interval
                # Prepare display
                if recent_i:
                    avg_i = sum_i / max(count_i,1)
                    avg_q = sum_q / max(count_q,1)
                    line1 = f"I last[{len(recent_i):02d}]: " + ' '.join(f"{v:03X}" for v in recent_i[-args.window:])
                    line2 = f"Q last[{len(recent_q):02d}]: " + ' '.join(f"{v:03X}" for v in recent_q[-args.window:])
                else:
                    line1 = 'I: (no data)'; line2 = 'Q: (no data)'
                lines = [f"Port {port}  Win={args.window}  Refresh={args.rate}Hz"]
                lines += [line1, line2]
                if args.spark and recent_i:
                    lines.append('I spark: ' + spark(recent_i))
                    lines.append('Q spark: ' + spark(recent_q))
                if args.stats and count_i:
                    # Window (recent) averages
                    wavg_i = sum(recent_i)/len(recent_i)
                    wavg_q = sum(recent_q)/len(recent_q)
                    lines.append(f"I min={min_i:03X} max={max_i:03X} avg(run)={avg_i:05.1f} avg(win)={wavg_i:05.1f}")
                    lines.append(f"Q min={min_q:03X} max={max_q:03X} avg(run)={avg_q:05.1f} avg(win)={wavg_q:05.1f}")
                # Clear screen region (simple approach)
                sys.stdout.write('\x1b[2J\x1b[H')
                sys.stdout.write('\n'.join(lines) + '\n')
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == '__main__':
    main()
