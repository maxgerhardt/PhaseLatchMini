#!/usr/bin/env python3
"""
USB CDC Tick Monitor

Purpose:
  - Open the STM32 CDC serial port
  - Capture and parse lines containing:
        EARLYTICK <t0> <t1> DT=<d>
        TICK=<n>
        HB ... (optional heartbeat line)
  - Compute:
        * Whether EARLYTICK delta > 0 (SysTick running early)
        * Live delta between successive TICK reports
        * Warn if TICK does not advance for a timeout window
  - Provide a rolling summary every N seconds.

Usage:
  python host_tick_monitor.py --port /dev/tty.usbmodemXXXX --baud 115200
  (Baud is ignored by CDC but kept for compatibility.)

Exit codes:
  0 success / normal user exit (Ctrl+C)
  2 no data received before timeout

"""
import argparse, sys, time, re, glob
try:
    import serial  # pyserial
except ImportError:
    print("ERROR: pyserial not installed: pip install pyserial", file=sys.stderr)
    sys.exit(1)

EARLY_RE = re.compile(r"EARLYTICK\s+(\d+)\s+(\d+)\s+DT=(\d+)")
TICK_RE = re.compile(r"TICK=(\d+)")
HB_RE = re.compile(r"HB ")


def auto_detect_port():
    cands = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
    return cands[0] if cands else None


def parse_args():
    p = argparse.ArgumentParser(description="Monitor HAL_GetTick output from device")
    p.add_argument('--port', help='Serial port (auto-detect if omitted)')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--timeout', type=float, default=10.0, help='Overall idle timeout (s)')
    p.add_argument('--stuck-secs', type=float, default=3.0, help='Seconds of no tick advance before warning')
    p.add_argument('--summary-interval', type=float, default=5.0, help='Seconds between summary lines')
    return p.parse_args()


def main():
    args = parse_args()
    port = args.port or auto_detect_port()
    if not port:
        print("ERROR: No serial port specified and auto-detect failed", file=sys.stderr)
        sys.exit(2)
    print(f"Opening {port} ...")
    ser = serial.Serial(port, args.baud, timeout=0.2)

    first_data_time = None
    last_any_time = time.time()
    last_tick_val = None
    last_tick_recv_time = None
    early_report = None
    stuck_reported = False
    next_summary = time.time() + args.summary_interval

    try:
        while True:
            line = ser.readline()
            now_host = time.time()
            if line:
                if first_data_time is None:
                    first_data_time = now_host
                last_any_time = now_host
                try:
                    s = line.decode('utf-8', errors='replace').strip()
                except Exception:
                    continue
                if not s:
                    continue
                print(s)
                m = EARLY_RE.match(s)
                if m:
                    t0, t1, dt = map(int, m.groups())
                    early_report = (t0, t1, dt)
                    if dt == 0:
                        print("WARNING: EARLY SysTick delta == 0 (SysTick not incrementing early)")
                    else:
                        print(f"INFO: EARLY SysTick delta {dt} (ok)")
                m = TICK_RE.match(s)
                if m:
                    tick = int(m.group(1))
                    if last_tick_val is not None and tick != last_tick_val:
                        interval = now_host - last_tick_recv_time if last_tick_recv_time else 0
                        # Basic estimation of tick frequency if interval reasonable
                        if interval > 0:
                            tick_diff = tick - last_tick_val
                            hz_est = tick_diff / interval
                            print(f"TICK_ADV tick_diff={tick_diff} interval={interval:.3f}s est={hz_est:.1f}Hz")
                        stuck_reported = False
                    last_tick_val = tick
                    last_tick_recv_time = now_host
                # Stuck detection
                if last_tick_val is not None and (now_host - last_tick_recv_time) > args.stuck_secs and not stuck_reported:
                    print(f"WARNING: No TICK advance for {args.stuck_secs} seconds (possible SysTick halt)")
                    stuck_reported = True
            else:
                # No line this poll; check idle timeout
                if (time.time() - last_any_time) > args.timeout:
                    print("ERROR: No data within timeout window")
                    sys.exit(2)
            if time.time() >= next_summary:
                etxt = "none"
                if early_report:
                    etxt = f"t0={early_report[0]} t1={early_report[1]} dt={early_report[2]}"
                ticktxt = f"last_tick={last_tick_val}" if last_tick_val is not None else "no_tick"
                print(f"SUMMARY early:{etxt} {ticktxt}")
                next_summary = time.time() + args.summary_interval
    except KeyboardInterrupt:
        print("Exiting on user request")
        sys.exit(0)

if __name__ == '__main__':
    main()
