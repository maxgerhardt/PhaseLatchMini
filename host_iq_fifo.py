#!/usr/bin/env python3
"""
Bridge STM32 CDC streamed 32-bit packed IQ (12-bit each) into a FIFO as 8-bit interleaved I/Q
for GQRX (File I/Q input) or other SDR tools.

Workflow:
 1. Create FIFO automatically (default: iq_fifo.iq). If it exists and is a pipe, reuse.
 2. Open CDC serial (/dev/tty.usbmodem*) and continuously read raw bytes.
 3. Parse little-endian 32-bit words; extract I = bits 0..11, Q = bits 12..23.
 4. Downscale to unsigned 8-bit via shift-right 4 (0..4095 -> 0..255) or optional dithering.
 5. Write interleaved [I0,Q0,I1,Q1,...] to FIFO (unbuffered).
 6. Optionally write a secondary file capture (raw 8-bit) for offline analysis.

GQRX Setup (File I/Q mode):
  - Sample rate: 100000
  - Format: Unsigned 8-bit I/Q (interleaved IQIQ...)
  - Uncheck 'Loop' so it treats as live; start after script begins writing.

If GQRX struggles to follow FIFO growth, run with --prebuf to accumulate some kB before GQRX start.

Usage:
  python host_iq_fifo.py --fifo iq_fifo.iq
  python host_iq_fifo.py --fifo iq_fifo.iq --capture out8.iq --prebuf 32768

Ctrl+C to stop; script reports counts and average throughput.
"""
import argparse, glob, os, sys, time, math, statistics, stat, fcntl, struct

try:
    import serial
except ImportError:
    print('ERROR: pyserial required (pip install pyserial)', file=sys.stderr)
    sys.exit(2)

DEFAULT_RATE = 100000  # complex samples/second (for info only)

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


def decode_block(buf, fmt, out_container):
    """Decode packed 32-bit IQ words into desired format.
    fmt: 'u8' -> append unsigned 8-bit interleaved IQ to bytearray
         'cf32' -> append little-endian float32 I,Q per sample
    out_container: bytearray for u8, or list of bytes objects for cf32 (to reduce per-append overhead)
    """
    if fmt == 'u8':
        ba = out_container  # alias
        for off in range(0, len(buf), 4):
            w = buf[off] | (buf[off+1] << 8) | (buf[off+2] << 16) | (buf[off+3] << 24)
            i12 = w & 0x0FFF
            q12 = (w >> 12) & 0x0FFF
            ba.append(i12 >> 4)
            ba.append(q12 >> 4)
    else:  # cf32
        pack = struct.pack
        for off in range(0, len(buf), 4):
            w = buf[off] | (buf[off+1] << 8) | (buf[off+2] << 16) | (buf[off+3] << 24)
            i12 = w & 0x0FFF
            q12 = (w >> 12) & 0x0FFF
            # Map 0..4095 -> -1.0 .. +1.0 (center at ~0)
            fi = (i12 - 2048) / 2048.0
            fq = (q12 - 2048) / 2048.0
            out_container.append(pack('<ff', fi, fq))


def main():
    ap = argparse.ArgumentParser(description='CDC -> FIFO 8-bit I/Q bridge for GQRX')
    ap.add_argument('--port', help='Explicit serial port path')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--fifo', default='iq_fifo.iq', help='FIFO path (created if absent)')
    ap.add_argument('--capture', help='Optional file to also dump 8-bit IQ stream')
    ap.add_argument('--timeout', type=float, default=0.01, help='Serial read timeout seconds')
    ap.add_argument('--prebuf', type=int, default=0, help='Bytes to accumulate before writing to FIFO (startup cushion)')
    ap.add_argument('--chunk', type=int, default=512, help='Serial read chunk size')
    ap.add_argument('--no-skip-text', action='store_true', help='Do not skip printable STAT-like chunks')
    ap.add_argument('--format', choices=['u8','cf32'], default='u8', help='Output sample format for FIFO (default u8)')
    args = ap.parse_args()

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

    partial = b''
    total_samples = 0
    total_bytes_out = 0
    start_time = time.time()
    prebuf_store = bytearray()

    try:
        while True:
            chunk = ser.read(args.chunk)
            if not chunk:
                continue
            if not args.no_skip_text:
                printable = sum(32 <= b < 127 for b in chunk)
                if printable/len(chunk) > ASCII_SKIP_THRESHOLD and b'STAT' in chunk:
                    continue
            partial += chunk
            if len(partial) < 4:
                continue
            cut = len(partial) - (len(partial) % 4)
            block = partial[:cut]
            partial = partial[cut:]
            if args.format == 'u8':
                outbuf = bytearray()
                decode_block(block, 'u8', outbuf)
                total_samples += len(outbuf)//2
            else:
                pieces = []
                decode_block(block, 'cf32', pieces)
                outbuf = b''.join(pieces)
                total_samples += len(outbuf)//8  # 8 bytes per complex sample
            if args.prebuf and len(prebuf_store) < args.prebuf:
                prebuf_store.extend(outbuf)
                if len(prebuf_store) >= args.prebuf:
                    os.write(fifo_fd, prebuf_store)
                    total_bytes_out += len(prebuf_store)
                    prebuf_store.clear()
                continue
            os.write(fifo_fd, outbuf)
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
        os.close(fifo_fd)
        elapsed = time.time()-start_time
        if elapsed>0:
            print(f"Final: samples={total_samples} avgRate={total_samples/elapsed:.1f} sps")

if __name__ == '__main__':
    main()
