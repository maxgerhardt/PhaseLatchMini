#!/usr/bin/env python3
"""
High-rate bulk-IN throughput measurement tool for the raw vendor class device.

Measures:
 - Sustained bytes/s and samples/s (assuming 32-bit words = I(16)|Q(16))
 - Packet rate (using read chunk size, default 4096 = multiples of 64)
 - Detects stalls (no data for --stall seconds) and alignment issues
 - Optionally writes raw stream to a file (e.g. for further processing)

Usage:
  pip install pyusb
  python host_throughput.py --seconds 5 --chunk 4096
  python host_throughput.py --outfile dump.bin --seconds 10

Notes:
  Full-speed USB theoretical max for bulk ~ 1.216 MB/s (19 * 64B * 1000 frames)
  Practical with Python user space may be slightly lower (~1.0-1.1 MB/s).
"""
import usb.core, usb.util, time, argparse, sys, statistics

EP_IN = 0x81

ap = argparse.ArgumentParser()
ap.add_argument('--vid', type=lambda x:int(x,0), default=0x0483)
ap.add_argument('--pid', type=lambda x:int(x,0), default=0x5741)
ap.add_argument('--seconds', type=float, default=5.0)
ap.add_argument('--chunk', type=int, default=4096, help='Host read size (multiple of 64 recommended)')
ap.add_argument('--timeout', type=int, default=1000, help='ms read timeout')
ap.add_argument('--outfile', '-o')
ap.add_argument('--stall', type=float, default=1.0, help='Stall detection threshold seconds (0=disable)')
ap.add_argument('--progress', action='store_true')
ap.add_argument('--no-store', action='store_true')
args = ap.parse_args()

dev = usb.core.find(idVendor=args.vid, idProduct=args.pid)
if not dev:
    print('Device not found', file=sys.stderr); sys.exit(1)
try:
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
except Exception:
    pass

dev.set_configuration()
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]
usb.util.claim_interface(dev, intf.bInterfaceNumber)

store = bytearray() if (args.outfile and not args.no_store) else None
outf = open(args.outfile,'wb') if args.outfile else None

start = time.time(); end = start + args.seconds
last_data = start
packet_sizes = []
bytes_total = 0
packets = 0
last_print = start
PRINT_IVL = 0.25

try:
    while True:
        now = time.time()
        if now >= end: break
        try:
            data = dev.read(EP_IN, args.chunk, timeout=args.timeout)
        except usb.core.USBError as e:
            if 'timed out' in str(e).lower():
                if args.stall and (time.time()-last_data) > args.stall:
                    print('\n[STALL] No data for %.2fs' % (time.time()-last_data))
                    break
                continue
            else:
                print('USBError', e, file=sys.stderr)
                break
        if not data:
            continue
        b = bytes(data)
        l = len(b)
        bytes_total += l
        packets += 1
        last_data = now
        packet_sizes.append(l)
        if outf: outf.write(b)
        if store is not None: store.extend(b)
        if args.progress and (now - last_print) >= PRINT_IVL:
            dt = now - start
            rate = bytes_total / dt
            sys.stdout.write('\r%8.1f KB/s  %7.1f kIQ/s  pkts=%6d  avgPkt=%.0f' % (rate/1024, (rate/4)/1000, packets, (sum(packet_sizes)/len(packet_sizes)) if packet_sizes else 0))
            sys.stdout.flush()
            last_print = now
finally:
    if outf: outf.close()
    try: usb.util.release_interface(dev, intf.bInterfaceNumber)
    except Exception: pass

dt = time.time() - start
if args.progress:
    print()
if dt <= 0: dt = 1e-6
rate = bytes_total / dt
print('Duration: %.3fs  Bytes: %d  Rate: %.1f KB/s (%.1f kIQ/s)' % (dt, bytes_total, rate/1024, (rate/4)/1000))
if packet_sizes:
    print('Packets: %d  Mean size: %.1f  Median: %.1f  Min: %d  Max: %d' % (len(packet_sizes), statistics.mean(packet_sizes), statistics.median(packet_sizes), min(packet_sizes), max(packet_sizes)))

if store is not None and store:
    # Alignment check
    if (len(store) % 4) != 0:
        print('WARNING: total bytes not multiple of 4 (trailing=%d)' % (len(store)%4))
    else:
        print('Alignment OK (multiple of 4 bytes)')
