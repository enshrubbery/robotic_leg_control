#!/usr/bin/env python3
"""
motor_ctl.py - interactive control for the 6-motor CubeMars rig.

Talks to the STM32 firmware over its UART command protocol (115200 8N1 on the
Nucleo VCP, default /dev/ttyACM1). Controls ONE motor per command line.

Command syntax (motor number first):

    <n> p<X> t<Y> v<Z>     move motor n:
                             p<X> = position CHANGE in rad (relative to current)
                             t<Y> = torque feedforward (Nm)
                             v<Z> = velocity feedforward (rad/s)
                           fields are optional and may appear in any order.
                           ('m' is accepted as an alias for 'p'.)
                           If p is given, t and v default to 0 when omitted
                           (so stale feedforwards never linger on a position move).

    <n> e        enable motor n (soft-start, then holds position)
    <n> r        passive feedback only (disable control)
    <n> o        set current position as the new zero (origin)
    <n> hold     re-hold motor n at its current position
    s            show latest state of all motors
    x            EMERGENCY STOP (makes all motors go limp, 0X)
    f            toggle live feedback printing on/off
    h            help
    q            quit

Examples:
    1 p0.5                 jog M1 +0.5 rad from where it is now
    4 p-0.2 v0.3           jog M4 (AK60) -0.2 rad with 0.3 rad/s feedforward
    3 e                    enable M3
    x                      stop everything
"""

import argparse
import math
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")

ENABLE_SETTLE_S = 1.4      # firmware soft-start takes ~1.2 s
FEEDBACK_WAIT_S = 5.0      # how long to wait for a fresh position before a move
HEARTBEAT_PERIOD_S = 0.25  # firmware watchdog expires after 1 second
DEG2RAD = 0.017453292519943295

# Jogs track a software setpoint, but it is never allowed to run more than this
# far ahead of the motor's real position. Keeps relative jogging deterministic
# (so + and - always mirror) while preventing setpoint runaway when the motor
# stalls or hits a joint limit -- and stays well under the firmware's 1.0 rad
# PID safety-kill threshold (main.c). Jog back always responds immediately.
MAX_FOLLOW_ERR = 0.5       # rad

# parses both feedback formats:
#   [M1] pos=0.247 rad  vel=...
#   [M4] pos=-96.2 deg  spd=...
FB_RE      = re.compile(r"\[M(\d+)\]\s+pos=(-?\d+\.?\d*)\s+(rad|deg)")
UNKNOWN_RE = re.compile(r"\[M0\] unknown CAN id=(\d+) ide=(\d+)")


class MotorLink:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.1
        self.ser.dtr = False   # do NOT reset the MCU on open
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.3)
        self.ser.reset_input_buffer()

        self.unknown_ids = {}  # can_id (int) -> {'ide': int, 'count': int}
        self.pos_rad = {}      # motor -> latest *unwrapped* position in radians
        self.last_line = {}    # motor -> latest raw feedback line
        self._fb_prev = {}     # motor -> previous wrapped feedback value (rad), for unwrapping
        self.enabled = set()   # motors we've enabled this session
        self.cmd_pos = {}      # motor -> software-tracked commanded setpoint (rad)
        self.print_fb = False
        self._scan_buf = None  # list when scan active, None otherwise
        self._lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._stop = False
        self._estopped = False   # True after estop(); heartbeat sends 0X until re-enabled
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._heartbeat = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._rx.start()
        self._heartbeat.start()

    # ---- IO ----------------------------------------------------------------
    def _reader(self):
        buf = b""
        while not self._stop:
            try:
                d = self.ser.read(256)
            except Exception:
                break
            if not d:
                continue
            buf += d
            while b"\n" in buf:
                chunk, buf = buf.split(b"\n", 1)
                line = chunk.decode(errors="replace").strip("\r")
                if not line:
                    continue
                m = FB_RE.search(line)
                if m:
                    n = int(m.group(1))
                    val = float(m.group(2))
                    raw = val * DEG2RAD if m.group(3) == "deg" else val
                    # Unwrap feedback so the tracked position stays continuous
                    # across the firmware's wrap boundary (e.g. +/-180 deg or
                    # 0-360 deg). Without this, a wrap makes fb jump ~2 pi rad,
                    # and the safety clamp in move() snaps the target to the
                    # wrong side of the wrap -- a near-full rotation for a tiny jog.
                    prev = self._fb_prev.get(n)
                    if prev is not None:
                        delta = raw - prev
                        while delta > math.pi:
                            raw -= 2 * math.pi
                            delta -= 2 * math.pi
                        while delta < -math.pi:
                            raw += 2 * math.pi
                            delta += 2 * math.pi
                    self._fb_prev[n] = raw
                    with self._lock:
                        self.pos_rad[n] = raw
                        self.last_line[n] = line
                u = UNKNOWN_RE.search(line)
                if u:
                    cid, ide = int(u.group(1)), int(u.group(2))
                    with self._lock:
                        if cid not in self.unknown_ids:
                            self.unknown_ids[cid] = {'ide': ide, 'count': 0}
                        self.unknown_ids[cid]['count'] += 1
                if self.print_fb:
                    print(line)
                if self._scan_buf is not None:
                    self._scan_buf.append(line)

    def _heartbeat_loop(self):
        while not self._stop:
            try:
                self.send("0X\n" if self._estopped else "0H\n")
            except Exception:
                break
            time.sleep(HEARTBEAT_PERIOD_S)

    def send(self, s):
        with self._tx_lock:
            self.ser.write(s.encode())
            self.ser.flush()

    def close(self):
        if self._stop:
            return
        try:
            self.send("0X\n")
            time.sleep(0.1)
        except Exception:
            pass
        self._stop = True
        self._rx.join(timeout=0.25)
        self._heartbeat.join(timeout=HEARTBEAT_PERIOD_S + 0.1)
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- helpers -----------------------------------------------------------
    def get_pos(self, n):
        with self._lock:
            return self.pos_rad.get(n)

    def get_unknown_ids(self):
        """Return a copy of the unknown CAN ID table: {can_id: {'ide': int, 'count': int}}."""
        with self._lock:
            return dict(self.unknown_ids)

    def wait_fresh_pos(self, n, timeout=FEEDBACK_WAIT_S):
        """Return motor n's latest known position (rad), waiting briefly if none yet."""
        p = self.get_pos(n)
        if p is not None:
            return p
        end = time.time() + timeout
        while time.time() < end:
            p = self.get_pos(n)
            if p is not None:
                return p
            time.sleep(0.02)
        return None

    def ensure_enabled(self, n):
        if n in self.enabled:
            return True
        if self._estopped:
            print(f"  M{n}: e-stop active, cannot enable")
            return False
        print(f"  M{n} not enabled -- enabling (soft-start ~{ENABLE_SETTLE_S:.1f}s)...")
        self.send(f"{n}E\n")
        time.sleep(ENABLE_SETTLE_S)
        self.enabled.add(n)
        return True

    # ---- high-level commands ----------------------------------------------
    def enable(self, n):
        self._estopped = False   # clear estop so heartbeat resumes normal keepalive
        self.send(f"{n}E\n")
        time.sleep(ENABLE_SETTLE_S)
        self.enabled.add(n)
        # request status to provoke feedback
        self.send(f"{n}s\n")
        time.sleep(0.5)
        # tiny move to elicit position feedback
        self.send(f"{n}p0.001\n")
        time.sleep(0.5)
        # seed the software jog setpoint from the motor's actual position
        self.cmd_pos[n] = self.get_pos(n) or 0.0
        print(f"  M{n} enabled (holding current position)")

    def passive(self, n):
        # Zero feedforwards before releasing — a live torque/velocity command
        # left active while switching to passive makes the motor spin/whine.
        self.send(f"{n}V0\n{n}T0\n{n}R\n")
        time.sleep(0.1)
        self.enabled.discard(n)
        # drop the setpoint so the next enable re-seeds from real feedback
        self.cmd_pos.pop(n, None)
        # reset unwrap state: motor may be moved freely while passive, so the
        # next position sample after re-enable is a new baseline
        self._fb_prev.pop(n, None)
        with self._lock:
            self.pos_rad.pop(n, None)
        print(f"  M{n} -> passive feedback (control disabled)")

    def origin(self, n):
        self.send(f"{n}O\n")
        time.sleep(0.1)
        # firmware just zeroed this motor's position, so the setpoint is 0 too
        self.cmd_pos[n] = 0.0
        # reset unwrap state: next feedback sample is a new baseline at ~0
        self._fb_prev.pop(n, None)
        with self._lock:
            self.pos_rad.pop(n, None)
        print(f"  M{n} origin set (current position is now 0)")

    def hold(self, n):
        cur = self.get_pos(n)
        if cur is None:
            cur = self.wait_fresh_pos(n)
        if cur is None:
            print(f"  M{n}: no feedback, cannot hold")
            return
        self.send(f"{n}V0\n{n}T0\n{n}P{cur:.4f}\n")
        # resync the jog setpoint to where we just told it to hold
        self.cmd_pos[n] = cur
        print(f"  M{n} holding at {cur:.4f} rad")

    def move(self, n, dpos=None, torque=None, vel=None):
        if not self.ensure_enabled(n):
            return
        if dpos is not None:
            # Jog relative to our software-tracked setpoint, NOT raw feedback.
            # Feedback is bursty/rate-limited, so using it as the baseline makes
            # successive jogs land on a stale sample and lose their sign. The
            # setpoint is seeded from real feedback on enable/zero/hold below.
            fb = self.get_pos(n)
            base = self.cmd_pos.get(n)
            if base is None:
                base = fb if fb is not None else self.wait_fresh_pos(n)
                if base is None:
                    print(f"  M{n}: no feedback received, cannot compute relative move")
                    return
            target = base + dpos
            # Clamp the setpoint to a safe band around the motor's real position
            # so it can't run away when the motor stalls / hits a joint limit.
            clamped = False
            if fb is not None:
                clamped_target = max(fb - MAX_FOLLOW_ERR, min(fb + MAX_FOLLOW_ERR, target))
                clamped = clamped_target != target
                target = clamped_target
            self.cmd_pos[n] = target
            v = vel if vel is not None else 0.0
            t = torque if torque is not None else 0.0
            # Torque is direction-relative: positive = assist motion, negative = resist.
            # Multiply by sign(dpos) so the user never has to think about motor polarity.
            t_sent = t * (1.0 if dpos >= 0 else -1.0)
            # order: feedforwards first, then position (P activates the PID)
            self.send(f"{n}V{v:.4f}\n{n}T{t_sent:.4f}\n{n}P{target:.4f}\n")
            if clamped and fb is not None:
                if abs(target - base) < 1e-6:
                    warn = (f"  *** STALLED: motor at {fb:+.4f} rad, "
                            f"can't reach setpoint {base:+.4f} rad ***")
                else:
                    warn = (f"  (clamped: motor at {fb:+.4f} rad, "
                            f"{abs(base - fb):.3f} rad behind setpoint)")
            elif abs(dpos) > 1.0:
                warn = "  (large move -- ramps gradually)"
            else:
                warn = ""
            print(f"  M{n}: {base:+.4f} -> {target:+.4f} rad  (d={dpos:+.3f}, "
                  f"v={v:.3f} rad/s, t={t:.3f} Nm){warn}")
        else:
            # no position change: just set whichever feedforwards were given
            if vel is not None:
                self.send(f"{n}V{vel:.4f}\n")
            if torque is not None:
                self.send(f"{n}T{torque:.4f}\n")
            print(f"  M{n}: set"
                  + (f" v={vel:.3f} rad/s" if vel is not None else "")
                  + (f" t={torque:.3f} Nm" if torque is not None else ""))

    def scan_can(self):
        """Probe IDs 1-127 and report any motor that responds with feedback."""
        with self._lock:
            known = set(self.last_line.keys())
        found = {}   # logical_n -> (probe_id, line)
        self._scan_buf = []
        print("  Scanning IDs 1-127 (~15s) ...")
        for cid in range(1, 128):
            self._scan_buf.clear()
            self.send(f"{cid}s\n")
            time.sleep(0.12)
            for line in list(self._scan_buf):
                m = FB_RE.search(line)
                if m:
                    n = int(m.group(1))
                    if n not in known and n not in found:
                        found[n] = (cid, line)
                        print(f"    probe {cid:3d} -> M{n} responded: {line}")
        self._scan_buf = None
        if found:
            print(f"  Found {len(found)} previously-silent motor(s)")
        else:
            print("  No new motors found (firmware may not forward IDs beyond its mapping)")

    def estop(self):
        self._estopped = True   # heartbeat now sends 0X until a motor is re-enabled
        self.send("0X\n")
        self.enabled.clear()
        # motors are limp; setpoints are meaningless until re-enabled
        self.cmd_pos.clear()
        print("  *** E-STOP sent (all motors limp) ***")

    def status(self):
        with self._lock:
            lines = dict(self.last_line)
            en = set(self.enabled)
        print("  --- motor status ---")
        for n in range(1, 7):
            tag = "EN " if n in en else "   "
            line = lines.get(n, "(no feedback)")
            print(f"  M{n} [{tag}] {line}")


FIELD_RE = re.compile(r"([pmtv])\s*(-?(?:\d+\.?\d*|\.\d+))", re.IGNORECASE)


def parse_fields(tokens):
    """Parse p/m/t/v fields from the remainder of a command line."""
    text = " ".join(tokens)
    out = {}
    for key, val in FIELD_RE.findall(text):
        k = key.lower()
        if k == "m":
            k = "p"
        out[k] = float(val)
    return out


def handle(link, raw):
    parts = raw.split()
    if not parts:
        return True
    head = parts[0].lower()

    if head in ("q", "quit", "exit"):
        return False
    if head in ("x", "stop", "estop"):
        link.estop()
        return True
    if head in ("s", "status"):
        link.status()
        return True
    if head in ("scan", "canscan"):
        link.scan_can()
        return True
    if head in ("f", "feedback"):
        link.print_fb = not link.print_fb
        print(f"  live feedback {'ON' if link.print_fb else 'OFF'}")
        return True
    if head in ("h", "help", "?"):
        print(__doc__)
        return True

    # motor-number-first commands
    if not head.isdigit():
        print("  ? unknown command. 'h' for help.")
        return True
    n = int(head)
    if not (1 <= n <= 6):
        print("  motor number must be 1-6")
        return True

    rest = parts[1:]
    if rest and rest[0].lower() in ("e", "enable"):
        link.enable(n); return True
    if rest and rest[0].lower() in ("r", "passive", "disable"):
        link.passive(n); return True
    if rest and rest[0].lower() in ("o", "origin", "zero"):
        link.origin(n); return True
    if rest and rest[0].lower() == "hold":
        link.hold(n); return True
    if rest and rest[0].lower() in ("i", "id") and len(rest) >= 2:
        cid = int(rest[1])
        link.send(f"{n}I{cid}\n")
        print(f"  M{n} CAN ID set to {cid}")
        return True

    f = parse_fields(rest)
    if not f:
        print("  nothing to do. e.g. '1 p0.5 t0.2 v0.1', or '1 e'")
        return True
    link.move(n, dpos=f.get("p"), torque=f.get("t"), vel=f.get("v"))
    return True


def autodetect_ports():
    """Return a list of candidate serial ports, best guesses first."""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    # Prefer USB CDC devices (STM32 Nucleo VCP shows up as ttyACM*).
    def score(p):
        dev = p.device
        if "ttyACM" in dev:
            return 0
        if "ttyUSB" in dev:
            return 1
        return 2
    return [p.device for p in sorted(ports, key=score)]


def main():
    ap = argparse.ArgumentParser(description="CubeMars 6-motor UART controller")
    ap.add_argument("--port", default=None,
                    help="serial port (default: autodetect ttyACM*/ttyUSB*)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    # Build the list of ports to try. An explicit --port is tried first/only;
    # otherwise scan for likely CubeMars/Nucleo serial devices.
    if args.port:
        candidates = [args.port]
    else:
        candidates = autodetect_ports()
        if not candidates:
            sys.exit("No serial ports found. Is the controller plugged in? "
                     "You can also pass --port explicitly.")
        print(f"Autodetect: trying {candidates}")

    link = None
    last_err = None
    for port in candidates:
        try:
            link = MotorLink(port, args.baud)
            break
        except Exception as e:
            last_err = e
            print(f"could not open {port}: {e}")

    if link is None or not link.ser.is_open:
        sys.exit(f"Could not open any serial port ({last_err}). "
                 "Pass --port to specify one.")

    port = link.ser.port
    print(f"Connected on {port} @ {args.baud}. Type 'h' for help, 'q' to quit.")
    print("Motors: M1 HipPitch(10) M2 HipRoll(11) M3 HipYaw(12) "
          "M4 Knee(106) M5 FootDiff1(104) M6 FootDiff2(105)")
    try:
        while True:
            try:
                raw = input("motor> ")
            except EOFError:
                break
            if not handle(link, raw):
                break
    except KeyboardInterrupt:
        print()
    finally:
        print("E-stopping and closing...")
        link.close()


if __name__ == "__main__":
    main()
