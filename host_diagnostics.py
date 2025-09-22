#!/usr/bin/env python3
"""
Host diagnostics tool for STM32 CDC / RAW USB streaming.

Features:
  - Auto-detect /dev/tty.usbmodem* port
  - Passive capture (banner / heartbeat sniff)
  - STATS sampler (two spaced reads + delta parsing)
  - Continuous monitor with periodic automatic STATS injection
  - Throughput test (byte rate over interval)
  - USB descriptor / interface summary (pyusb, optional)

Usage examples:
  python host_diagnostics.py capture --secs 3
  python host_diagnostics.py stats
  python host_diagnostics.py monitor --stats-interval 1.0
  python host_diagnostics.py throughput --secs 5
  python host_diagnostics.py usbinfo

Exits non‑zero on fatal errors (no port, etc.).
"""
import argparse, glob, sys, time, re, json

try:
    import serial  # pyserial
except ImportError:
    print("ERROR: pyserial not installed: pip install pyserial", file=sys.stderr)
    sys.exit(2)

def find_port(explicit=None):
    if explicit:
        return explicit
    ports = sorted(glob.glob('/dev/tty.usbmodem*'))
    if not ports:
        raise SystemExit("No /dev/tty.usbmodem* device found")
    return ports[0]

STAT_RE = re.compile(r'STAT\s+TA=(\d+)\s+TB=(\d+)\s+TC=(\d+)\s+WD=(\d+)\s+P=(\d+).*?LH=(\d+)\s+E1=(\d+)\s+EB=(\d+)(?:\s+FBF=(\d+))?')

FIELDS = ['TA','TB','TC','WD','P','LH','E1','EB','FBF']

def parse_stat(line):
    m = STAT_RE.search(line)
    if not m:
        return None
    groups = list(m.groups())
    if groups[-1] is None:
        groups[-1] = '0'  # default FBF if absent
    return {k:int(v) for k,v in zip(FIELDS, groups)}

def cmd_capture(args):
    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=0.05)
    end = time.time()+args.secs
    buf=[]
    while time.time()<end:
        d=ser.read(512)
        if d:
            text=d.decode(errors='ignore')
            buf.append(text)
            if not args.quiet:
                sys.stdout.write(text)
                sys.stdout.flush()
    ser.close()
    if args.json:
        print(json.dumps({'captured':' '.join(buf)}, indent=2))


def cmd_stats(args):
    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=0.2)
    def one():
        ser.write(b'STATS\n')
        time.sleep(0.15)
        resp = ser.read(800).decode(errors='ignore')
        print(resp.strip())
        stat = parse_stat(resp) or {}
        return stat
    s1 = one()
    time.sleep(args.delay)
    s2 = one()
    ser.close()
    if s1 and s2:
        delta = {k: s2.get(k,0)-s1.get(k,0) for k in FIELDS}
        print("Parsed #1:", s1)
        print("Parsed #2:", s2)
        print("Delta:", delta)
        # Heuristic interpretation
        hints=[]
        if delta['E1']>0: hints.append('Endpoint IN completions occurring (hardware active).')
        else: hints.append('No EP1 completions detected (E1 static).')
        if delta['TC']>0: hints.append('CDC completion hook firing (TC advancing).')
        else: hints.append('CDC completion hook NOT firing (TC static).')
        if delta['TA']==0: hints.append('No new CDC transmit attempts (TA static).')
        if delta['LH']==0: hints.append('Loop heart static - task not running or blocked.')
        if s2.get('FBF',0)==1: hints.append('Fallback class data in use.')
        print("Hints:")
        for h in hints: print(" -", h)


def cmd_monitor(args):
    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=0.05)
    next_stats = time.time()+args.stats_interval if args.stats_interval>0 else None
    try:
        while True:
            d=ser.read(512)
            if d:
                sys.stdout.write(d.decode(errors='ignore'))
                sys.stdout.flush()
            if next_stats and time.time()>=next_stats:
                ser.write(b'STATS\n')
                time.sleep(0.12)
                resp=ser.read(800).decode(errors='ignore')
                print(resp.strip())
                next_stats=time.time()+args.stats_interval
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def cmd_throughput(args):
    port = find_port(args.port)
    ser = serial.Serial(port, args.baud, timeout=0)
    start=time.time(); end=start+args.secs
    count=0
    while time.time()<end:
        d=ser.read(4096)
        count+=len(d)
    ser.close()
    rate = count/args.secs/1024.0
    print("Bytes: {}  Rate: {:.1f} KB/s".format(count, rate))


def cmd_usbinfo(args):
    try:
        import usb.core, usb.util
    except ImportError:
        print('pyusb not installed: pip install pyusb', file=sys.stderr)
        sys.exit(3)
    # Try both original and modified PID possibilities
    for pid in (0x5741,0x5750):
        dev = usb.core.find(idVendor=0x0483, idProduct=pid)
        if dev:
            print(f"Found VID=0x0483 PID=0x{pid:04X}")
            print("Configurations:", dev.bNumConfigurations)
            for cfg in dev:
                print(f" Config {cfg.bConfigurationValue} TotalLen={cfg.wTotalLength}")
                for intf in cfg:
                    print(f"  IF#{intf.bInterfaceNumber} Alt={intf.bAlternateSetting} EPs={len(intf.endpoints())}")
                    for ep in intf.endpoints():
                        print(f"    EP 0x{ep.bEndpointAddress:02X} type={(ep.bmAttributes & 3)} maxPkt={ep.wMaxPacketSize}")
            break
    else:
        print("No matching USB device found (VID=0x0483 PIDs 0x5741/0x5750)")


def build_parser():
    p = argparse.ArgumentParser(description='STM32 USB CDC/RAW host diagnostics')
    p.add_argument('--port', help='Explicit serial port path')
    p.add_argument('--baud', type=int, default=115200)
    sub = p.add_subparsers(dest='cmd', required=True)

    sc = sub.add_parser('capture', help='Passive capture of text output')
    sc.add_argument('--secs', type=float, default=3.0)
    sc.add_argument('--quiet', action='store_true')
    sc.add_argument('--json', action='store_true')
    sc.set_defaults(func=cmd_capture)

    ss = sub.add_parser('stats', help='Send two STATS and show deltas')
    ss.add_argument('--delay', type=float, default=0.8)
    ss.set_defaults(func=cmd_stats)

    sm = sub.add_parser('monitor', help='Continuous monitor with periodic STATS')
    sm.add_argument('--stats-interval', type=float, default=1.0)
    sm.set_defaults(func=cmd_monitor)

    st = sub.add_parser('throughput', help='Measure raw byte rate')
    st.add_argument('--secs', type=float, default=5.0)
    st.set_defaults(func=cmd_throughput)

    su = sub.add_parser('usbinfo', help='List USB descriptor details via pyusb')
    su.set_defaults(func=cmd_usbinfo)

    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)

if __name__ == '__main__':
    main()
