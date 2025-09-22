#!/usr/bin/env python3
"""
Simple bulk-IN reader using PyUSB for the raw vendor device.
Finds device (default VID=0x0483 PID=0x5750), claims interface 0, reads endpoint 0x81 and writes raw bytes to a file or stdout.

Usage:
  pip install pyusb
  python host_raw_capture.py --outfile samples.iq --seconds 10

List devices:
  python host_raw_capture.py --list [--vid 0x0483]
"""
import argparse
import sys
import time
import usb.core
import usb.util
import signal

# Suppress BrokenPipe noisy traceback when piping to head/tail
def _silent_broken_pipe_hook(exctype, value, tb):
    if exctype is BrokenPipeError:
        try:
            sys.stderr.flush()
        except Exception:
            pass
        return
    # Fall back to default for other exceptions
    sys.__excepthook__(exctype, value, tb)

sys.excepthook = _silent_broken_pipe_hook

def _handle_sigpipe(signum, frame):
    # Immediate quiet exit on SIGPIPE
    try:
        sys.exit(0)
    except SystemExit:
        raise

signal.signal(signal.SIGPIPE, _handle_sigpipe)

EP_IN = 0x81

parser = argparse.ArgumentParser()
parser.add_argument('--vid', type=lambda x: int(x,0), default=0x0483, help='Vendor ID (default 0x0483)')
parser.add_argument('--pid', type=lambda x: int(x,0), default=0x5741, help='Product ID (default 0x5750)')
parser.add_argument('--list', action='store_true', help='List matching devices and exit')
parser.add_argument('--outfile', '-o', default=None, help='Write raw bytes to this file (default: stdout)')
parser.add_argument('--seconds', '-s', type=float, default=10.0, help='How long to record (seconds)')
parser.add_argument('--timeout', '-t', type=int, default=1000, help='USB read timeout (ms)')
parser.add_argument('--chunk', '-c', type=int, default=64, help='Read chunk size in bytes')
parser.add_argument('--quiet', '-q', action='store_true', help='Suppress banner and final status (useful for pipelines)')
args = parser.parse_args()

vid, pid = args.vid, args.pid

if args.list:
    devs = list(usb.core.find(find_all=True, idVendor=vid)) if args.vid else list(usb.core.find(find_all=True))
    if not devs:
        print('No devices found for VID filter' if args.vid else 'No USB devices found (unexpected)')
        sys.exit(0)
    for d in devs:
        try:
            print('Device: VID=0x%04X PID=0x%04X bDeviceClass=0x%02X' % (d.idVendor, d.idProduct, d.bDeviceClass))
        except Exception as e:
            print('Device (error reading attrs):', e)
    sys.exit(0)

dev = usb.core.find(idVendor=vid, idProduct=pid)
if dev is None:
    print('Device not found: VID=0x%04X PID=0x%04X' % (vid, pid), file=sys.stderr)
    sys.exit(1)

# Detach kernel driver if necessary
try:
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
except Exception:
    pass

dev.set_configuration()
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

# Claim interface 0
usb.util.claim_interface(dev, intf.bInterfaceNumber)

out = sys.stdout.buffer if args.outfile is None else open(args.outfile, 'wb')
end = time.time() + args.seconds
if not args.quiet:
    print('Starting capture (VID=0x%04X PID=0x%04X), writing to' % (vid, pid), 'stdout' if args.outfile is None else args.outfile, file=sys.stderr)
try:
    while time.time() < end:
        try:
            data = dev.read(EP_IN, args.chunk, timeout=args.timeout)
            out.write(bytes(data))
            try:
                out.flush()
            except BrokenPipeError:
                # Upstream pipe closed (e.g., head exited); stop cleanly
                break
        except usb.core.USBError as e:
            if e.errno == 110 or 'timed out' in str(e).lower():
                continue
            else:
                print('USBError', e, file=sys.stderr)
                break
finally:
    try:
        usb.util.release_interface(dev, intf.bInterfaceNumber)
    except Exception:
        pass
    if args.outfile is not None:
        out.close()

if not args.quiet:
    print('Done', file=sys.stderr)
