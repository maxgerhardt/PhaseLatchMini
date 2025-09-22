#!/usr/bin/env python3
"""
Simple Mac host test for STM32F103 CDC IQ streamer.

Features:
- Auto-detect /dev/tty.usbmodem* (or provide --port)
- Sends START command
- Reads IQ 32-bit words (I=lower16, Q=upper16 unsigned) and converts to signed centered values
- Prints basic stats and optionally writes raw complex float32 to a file for GNU Radio / GQRX

Usage examples:
  python3 host_test.py --seconds 5 --out iq.f32
  python3 host_test.py --port /dev/tty.usbmodem123456 --seconds 2

To visualize quickly with numpy/matplotlib:
  python3 - <<'EOF'
import numpy as np
x = np.fromfile('iq.f32', dtype=np.float32).view(np.complex64)
print('Samples:', x.size)
EOF
"""
import sys, time, struct, glob, argparse, os, signal

try:
    import serial
except ImportError:
    print("pyserial needed: pip install pyserial")
    sys.exit(1)

parser = argparse.ArgumentParser(description="STM32 CDC IQ capture utility with graceful interrupt handling")
parser.add_argument('--port', help='Serial port (auto if omitted)')
parser.add_argument('--baud', type=int, default=115200, help='Ignored by CDC but required by pyserial')
parser.add_argument('--seconds', type=float, default=3.0, help='Capture duration (ignored if --max-bytes or --pairs used)')
parser.add_argument('--out', help='Optional output file (float32 interleaved I,Q)')
parser.add_argument('--max-bytes', type=int, default=0, help='Stop after this many payload bytes (0=disabled)')
parser.add_argument('--pairs', type=int, default=0, help='Stop after this many IQ pairs (0=disabled)')
parser.add_argument('--progress', action='store_true', help='Show live progress bar / stats')
parser.add_argument('--quiet', action='store_true', help='Suppress periodic rate prints')
parser.add_argument('--read-timeout', type=float, default=0.2, help='Serial read timeout seconds (default 0.2, smaller = more responsive Ctrl+C)')
parser.add_argument('--idle-exit', type=float, default=0.0, help='Exit if no data received for this many seconds (0=disabled)')
parser.add_argument('--no-store', action='store_true', help='Do not store raw bytes (reduces RAM). Stats limited; no file output.')
parser.add_argument('--raw-dump', help='Optional raw binary output file (32-bit words) written streaming (even with --no-store)')
parser.add_argument('--diagnose', action='store_true', help='Diagnostic mode: probe command endings, poll STATS, print early bytes')
parser.add_argument('--raw-hex', nargs='?', const=64, type=int, default=0,
                    help='Print hex dump of first N bytes (default 64 if value omitted)')
parser.add_argument('--show-ascii', action='store_true', help='Show printable ASCII of incoming bytes during capture')
args = parser.parse_args()

if not args.port:
    cands = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
    if not cands:
        print('No USB CDC ports found')
        sys.exit(1)
    args.port = cands[0]

print(f'Using port: {args.port}')

ser = serial.Serial(args.port, args.baud, timeout=args.read_timeout)
# Give device time after opening
time.sleep(0.5)

# Flush any banner
ser.reset_input_buffer()

def send(cmd, ending=b'\r'):
    ser.write(cmd + ending)
    ser.flush()

if args.diagnose:
    print('[DIAG] Probing line endings for STATS...')
    for ending in (b'\r', b'\n', b'\r\n'):
        send(b'STATS', ending)
        time.sleep(0.05)
    print('[DIAG] Sending START with CR, LF, CRLF')
    for ending in (b'\r', b'\n', b'\r\n'):
        send(b'START', ending)
        time.sleep(0.05)
else:
    send(b'START')
start_time = time.time()
raw_bytes = bytearray() if not args.no_store else None
raw_count = 0  # total bytes seen (even if not stored)

TARGET_END = start_time + args.seconds
max_bytes = args.max_bytes if args.max_bytes > 0 else None
target_pairs = args.pairs if args.pairs > 0 else None

def format_rate(bytes_so_far, t0):
    dt = max(1e-6, time.time()-t0)
    bps = bytes_so_far/dt
    return f"{bps/1024:.1f} KB/s ({(bps/4)/1000:.1f} kIQ/s)"

last_update = 0.0
update_interval = 0.25

raw_out_f = None
if args.raw_dump:
    raw_out_f = open(args.raw_dump, 'wb')

last_data_time = time.time()
interrupted = False
hex_remaining = args.raw_hex

def handle_sigint(signum, frame):
    global interrupted
    interrupted = True

signal.signal(signal.SIGINT, handle_sigint)

try:
    while True:
        current_len = raw_count if args.no_store else len(raw_bytes)
        if max_bytes and current_len >= max_bytes:
            break
        if target_pairs and (current_len//4) >= target_pairs:
            break
        if not max_bytes and not target_pairs and time.time() >= TARGET_END:
            break
        if interrupted:
            if args.progress:
                print("\n[INT] Ctrl+C received, stopping...")
            break
        chunk = ser.read(1024)
        if chunk:
            last_data_time = time.time()
            raw_count += len(chunk)
            if hex_remaining > 0:
                display = chunk[:hex_remaining]
                print('\n[HEX]', ' '.join(f'{b:02X}' for b in display))
                hex_remaining -= len(display)
            if args.show_ascii:
                printable = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
                print(f'\n[ASC] {printable}')
            if raw_out_f:
                raw_out_f.write(chunk)
            if raw_bytes is not None:
                raw_bytes.extend(chunk)
            if args.diagnose and (raw_count < 256) and (b'STATS' in chunk or b'OK' in chunk or b'?' in chunk):
                print('[DIAG] Echo/Response fragment:', chunk)
        elif args.idle_exit > 0 and (time.time() - last_data_time) > args.idle_exit:
            if args.progress:
                print("\n[IDLE] No data, idle timeout reached")
            break
        if args.progress and (time.time()-last_update) > update_interval:
            pairs = (raw_count if args.no_store else len(raw_bytes))//4
            goal_desc = []
            cur_bytes = raw_count if args.no_store else len(raw_bytes)
            if max_bytes: goal_desc.append(f"{cur_bytes}/{max_bytes}B")
            if target_pairs: goal_desc.append(f"{pairs}/{target_pairs}pr")
            if not goal_desc: goal_desc.append(f"{pairs}pr")
            rate = format_rate(cur_bytes, start_time)
            print("\r[CAP] " + ' '.join(goal_desc) + " " + rate + '    ', end='', flush=True)
        # Loop continues
finally:
    # Always attempt to stop the device
    try:
        ser.write(b'STOP\r')
        ser.flush()
    except Exception:
        pass
    if raw_out_f:
        raw_out_f.close()

if args.progress:
    print()  # newline after progress / interrupt lines

total_bytes = raw_count if args.no_store else len(raw_bytes)
print(f'Received {total_bytes} bytes raw in {time.time()-start_time:.2f}s ({format_rate(total_bytes, start_time)})')

# Data should be multiples of 4
if raw_bytes is None:
    if args.out:
        print('Cannot write float32 output with --no-store enabled')
    print('Streaming mode (no-store) finished; skipped detailed stats.')
    sys.exit(0)

usable = len(raw_bytes) - (len(raw_bytes) % 4)
if usable != len(raw_bytes):
    print(f'Trimming {len(raw_bytes)-usable} trailing bytes to align to 32-bit words')
raw_bytes = raw_bytes[:usable]

word_count = usable // 4
print(f'Words: {word_count}  Pairs: {word_count}')

if word_count == 0:
    sys.exit(0)

# Unpack little-endian 32-bit words (each holds two 16-bit lanes with 12-bit right-aligned samples)
words = struct.unpack('<' + 'I'*word_count, raw_bytes)
I = []  # centered float-ready integers (scaled later)
Q = []
for w in words:
    lane_i =  w        & 0xFFFF  # lower half-word
    lane_q = (w >> 16) & 0xFFFF  # upper half-word
    raw12_i = lane_i & 0x0FFF
    raw12_q = lane_q & 0x0FFF
    # Center at 2048 (12-bit midscale) giving signed range roughly [-2048,+2047]
    I.append(raw12_i - 2048)
    Q.append(raw12_q - 2048)

# Optional write float32 IQ interleaved (normalized to [-1,+1))
if args.out:
    import array
    scale = 2048.0  # 12-bit midscale
    floats = array.array('f')
    for a,b in zip(I,Q):
        floats.append(a/scale)
        floats.append(b/scale)
    with open(args.out, 'wb') as f:
        floats.tofile(f)
    print(f'Wrote {len(I)} IQ pairs to {args.out} (12-bit centered scaling)')

# Basic stats
import math
mean_i = sum(I)/len(I)
mean_q = sum(Q)/len(Q)
var_i = sum((x-mean_i)**2 for x in I)/len(I)
var_q = sum((x-mean_q)**2 for x in Q)/len(Q)
print(f'I mean={mean_i:.1f} rms={math.sqrt(var_i):.1f}  Q mean={mean_q:.1f} rms={math.sqrt(var_q):.1f}')

print('Done')

# Notes:
#  - Press Ctrl+C once for graceful stop (sends STOP). Press twice quickly to force if shell still busy.
#  - Use --read-timeout smaller for more responsiveness, or --idle-exit to auto-stop when stream dies.
#  - Use --no-store + --raw-dump for long captures without large RAM usage.
