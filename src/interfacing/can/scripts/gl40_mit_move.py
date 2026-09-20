#!/usr/bin/env python3
"""Bench-move ONE CubeMars GL II-drive motor (e.g. GL40 II) in MIT mode over raw SocketCAN.

Talks straight to ``can0`` (python stdlib ``socket.AF_CAN``) -- no ROS, no can_node. can_node's
MIT_CONTROL path cannot drive a GL II drive today (wrong byte order in humanoid.dbc, extended
instead of standard CAN IDs, no GL40 profile, no enter/exit-motor-mode frames), so this script
implements the protocol from the CubeMars "Gimbal Motor Drive User Manual V1.0 -- For GL II"
section 5 directly. It is deliberately a single-motor bench tool, not an arm controller.

Protocol (GL II manual §5, standard 11-bit frames, 1 Mbps):
  command  ID = (mode<<8)|node_id, MIT mode -> node_id itself
           data = [p>>8, p, v>>4, (v&F)<<4|kp>>8, kp, kd>>4, (kd&F)<<4|t>>8, t]
  feedback ID = master id (default 0)
           data = [err<<4|id&F, p>>8, p, v>>4, (v&F)<<4|t>>8, t, drive_temp, motor_temp]
  special  FF FF FF FF FF FF FF FC/FD/FE/FB = enter / exit motor mode / set zero / clear errors
The P/V/T ranges used to pack floats are PER-DRIVE settings (upper computer -> parameter page);
the defaults are +-12.5 rad / +-200 / +-10 N.m. If the drive is configured differently, pass
--p-max/--v-max/--t-max or every position in this script is wrong.

Safety (see .claude/skills/real-hardware-safety/SKILL.md): reads the current position before
moving, holds there first, ramps the setpoint slowly, refuses gains that could exceed
--max-torque at the commanded error, aborts on torque / tracking-error / temperature / error
code / lost feedback, and sends "exit motor mode" (motor goes limp) on Ctrl-C or any failure.
Nothing here replaces a HARDWARE E-STOP on the motor supply.

Examples (inside the interfacing container; can0 is brought up by can_node / setup_can.sh)::

  S=/root/ament_ws/src/interfacing/can/scripts/gl40_mit_move.py
  sudo python3 $S --selftest                      # packing vs. manual example, no bus
  sudo python3 $S --id 22 --monitor               # zero-torque; turn shaft by hand, check rad scale
  sudo python3 $S --id 22 --deg 40 --dry-run      # print the frames that would be sent
  sudo python3 $S --id 22 --deg 40                # +40 deg from current position, then go limp
  sudo python3 $S --id 22 --deg 40 --hold         # ...and keep holding until Ctrl-C
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
# Protocol constants (GL II manual §5)
# ---------------------------------------------------------------------------

FRAME_ENTER_MOTOR_MODE = bytes([0xFF] * 7 + [0xFC])
FRAME_EXIT_MOTOR_MODE = bytes([0xFF] * 7 + [0xFD])
FRAME_SET_ZERO = bytes([0xFF] * 7 + [0xFE])  # not used here; documented for completeness
FRAME_CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

ERR_NAMES = {
    0x0: "Disable",
    0x1: "Enable",
    0x8: "Over-voltage",
    0x9: "Under-voltage",
    0xA: "Over-current",
    0xB: "MOS over-temperature",
    0xC: "Motor winding over-temperature",
    0xD: "Communication loss",
    0xE: "Overload",
}
ERR_OK = {0x0, 0x1}

# Linux SocketCAN frame: can_id (u32), can_dlc (u8), 3 pad bytes, 8 data bytes.
_CAN_FRAME_FMT = "=IB3x8s"
_CAN_FRAME_SIZE = struct.calcsize(_CAN_FRAME_FMT)

DEG = math.pi / 180.0


@dataclass(frozen=True)
class MitRanges:
    """Float<->fixed-point spans. P/V/T are per-drive settings; KP/KD are fixed by the drive."""

    p_max: float = 12.5   # rad,   pos in [-p_max, p_max]
    v_max: float = 200.0  # (manual: "r/s"), vel in [-v_max, v_max]
    t_max: float = 10.0   # N.m,   t_ff in [-t_max, t_max]
    kp_max: float = 500.0  # N.m/rad,   kp in [0, kp_max]
    kd_max: float = 5.0    # N.m.s/rad, kd in [0, kd_max]


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Manual §5.1 float_to_uint: clamp, then (x - min) * (2^bits / span)."""
    span = x_max - x_min
    x = min(max(x, x_min), x_max)
    return int((x - x_min) * ((1 << bits) / span))


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """Manual §5.4 uint_to_float: x * span / (2^bits - 1) + min."""
    span = x_max - x_min
    return float(x_int) * span / float((1 << bits) - 1) + x_min


def pack_mit(p: float, v: float, kp: float, kd: float, t: float, r: MitRanges) -> bytes:
    """MIT command payload, byte order from manual §5.1 ctrl_motor()."""
    p_i = float_to_uint(p, -r.p_max, r.p_max, 16)
    v_i = float_to_uint(v, -r.v_max, r.v_max, 12)
    kp_i = float_to_uint(kp, 0.0, r.kp_max, 12)
    kd_i = float_to_uint(kd, 0.0, r.kd_max, 12)
    t_i = float_to_uint(t, -r.t_max, r.t_max, 12)
    return bytes([
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0xF) << 4) | ((kp_i >> 8) & 0xF),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0xF) << 4) | ((t_i >> 8) & 0xF),
        t_i & 0xFF,
    ])


@dataclass(frozen=True)
class Feedback:
    id_nibble: int
    err: int
    pos: float        # rad
    vel: float        # manual units (r/s)
    torque: float     # N.m
    drive_temp: int   # degC
    motor_temp: int   # degC

    @property
    def err_name(self) -> str:
        return ERR_NAMES.get(self.err, f"unknown(0x{self.err:X})")

    def __str__(self) -> str:
        return (f"id&F={self.id_nibble} err={self.err_name} pos={self.pos:+.4f} rad "
                f"({self.pos / DEG:+.1f} deg) vel={self.vel:+.2f} tau={self.torque:+.3f} N.m "
                f"drive={self.drive_temp}C motor={self.motor_temp}C")


def unpack_feedback(data: bytes, r: MitRanges) -> Feedback:
    """Feedback payload, manual §5.4 motor_receive()."""
    if len(data) < 8:
        raise ValueError(f"feedback frame too short: {data.hex()}")
    err = data[0] >> 4
    id_nibble = data[0] & 0xF
    pos_i = (data[1] << 8) | data[2]
    vel_i = (data[3] << 4) | (data[4] >> 4)
    t_i = ((data[4] & 0xF) << 8) | data[5]
    return Feedback(
        id_nibble=id_nibble,
        err=err,
        pos=uint_to_float(pos_i, -r.p_max, r.p_max, 16),
        vel=uint_to_float(vel_i, -r.v_max, r.v_max, 12),
        torque=uint_to_float(t_i, -r.t_max, r.t_max, 12),
        drive_temp=struct.unpack("b", bytes([data[6]]))[0],
        motor_temp=struct.unpack("b", bytes([data[7]]))[0],
    )


# ---------------------------------------------------------------------------
# Bus
# ---------------------------------------------------------------------------

class CanBus:
    """Minimal raw SocketCAN wrapper (standard 11-bit frames only)."""

    def __init__(self, iface: str, dry_run: bool = False):
        self.iface = iface
        self.dry_run = dry_run
        self.sock: Optional[socket.socket] = None
        if dry_run:
            return
        if not os.path.exists(f"/sys/class/net/{iface}"):
            sys.exit(f"CAN interface {iface} does not exist. Bring it up first, e.g. inside the "
                     f"interfacing container:\n  sudo /root/ament_ws/src/interfacing/can/scripts/"
                     f"setup_can.sh /dev/canable {iface} -s8\n(or just start can_node: "
                     f"ros2 launch can can.launch.py)")
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)

    def send(self, can_id: int, data: bytes) -> None:
        if not (0 <= can_id <= 0x7FF):
            raise ValueError(f"standard CAN id out of range: 0x{can_id:X}")
        if self.dry_run:
            print(f"  [dry-run] TX {self.iface} {can_id:03X}#{data.hex().upper()}")
            return
        assert self.sock is not None
        frame = struct.pack(_CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b"\0"))
        self.sock.send(frame)

    def recv(self, timeout_s: float) -> Optional[Tuple[int, bytes]]:
        """Return (can_id, data) or None on timeout. Extended/RTR/error frames are skipped."""
        if self.dry_run:
            return None
        assert self.sock is not None
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                raw = self.sock.recv(_CAN_FRAME_SIZE)
            except BlockingIOError:
                time.sleep(min(0.0005, remaining))
                continue
            can_id, dlc, data = struct.unpack(_CAN_FRAME_FMT, raw)
            if can_id & (socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
                continue
            return can_id & socket.CAN_SFF_MASK, data[:dlc]

    def drain(self) -> None:
        while self.recv(0.0) is not None:
            pass

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None


# ---------------------------------------------------------------------------
# Motor link + safety
# ---------------------------------------------------------------------------

class Abort(RuntimeError):
    """Raised to stop the move; the caller's finally-block frees the motor."""


@dataclass(frozen=True)
class SafetyLimits:
    max_torque: float          # N.m, refuse gains / abort feedback above this
    max_track_err: float       # rad, abort if |feedback.pos - setpoint| exceeds this
    max_temp: int              # degC, drive or motor
    feedback_timeout: float    # s of consecutive silence before abort
    max_setpoint_vel: float    # rad/s, upper bound on how fast the setpoint may move


class Motor:
    def __init__(self, bus: CanBus, motor_id: int, master_id: int, ranges: MitRanges,
                 limits: SafetyLimits):
        if not (1 <= motor_id <= 0xFF):
            raise ValueError("motor id must be 1..255")
        self.bus = bus
        self.id = motor_id
        self.master_id = master_id
        self.r = ranges
        self.lim = limits
        self.last_fb: Optional[Feedback] = None
        self.last_fb_time = time.monotonic()

    # --- raw frames ------------------------------------------------------
    def enter_motor_mode(self) -> None:
        self.bus.send(self.id, FRAME_ENTER_MOTOR_MODE)

    def exit_motor_mode(self) -> None:
        self.bus.send(self.id, FRAME_EXIT_MOTOR_MODE)

    def command(self, p: float, v: float, kp: float, kd: float, t: float) -> None:
        if abs(p) > self.r.p_max:
            raise Abort(f"refusing setpoint {p:.3f} rad outside +-{self.r.p_max} rad")
        self.bus.send(self.id, pack_mit(p, v, kp, kd, t, self.r))

    # --- feedback --------------------------------------------------------
    def read_feedback(self, timeout_s: float) -> Optional[Feedback]:
        """Newest feedback frame from this motor within timeout, or None."""
        deadline = time.monotonic() + timeout_s
        fb: Optional[Feedback] = None
        while True:
            remaining = deadline - time.monotonic()
            got = self.bus.recv(max(remaining, 0.0))
            if got is None:
                break
            can_id, data = got
            if can_id != self.master_id or len(data) < 8 or (data[0] & 0xF) != (self.id & 0xF):
                continue
            fb = unpack_feedback(data, self.r)
            # keep draining in case several replies queued up; the last one is the freshest
            deadline = min(deadline, time.monotonic() + 0.0005)
        if fb is not None:
            self.last_fb = fb
            self.last_fb_time = time.monotonic()
        return fb

    def check(self, fb: Optional[Feedback], setpoint: Optional[float]) -> None:
        """Abort on anything the safety skill says must stop the motor."""
        if fb is None:
            if time.monotonic() - self.last_fb_time > self.lim.feedback_timeout:
                raise Abort(f"no feedback from motor {self.id} for "
                            f"{self.lim.feedback_timeout * 1e3:.0f} ms")
            return
        if fb.err not in ERR_OK:
            raise Abort(f"motor reports error: {fb.err_name}")
        if abs(fb.torque) > self.lim.max_torque:
            raise Abort(f"feedback torque {fb.torque:+.3f} N.m exceeds {self.lim.max_torque} N.m")
        if max(fb.drive_temp, fb.motor_temp) > self.lim.max_temp:
            raise Abort(f"temperature {max(fb.drive_temp, fb.motor_temp)} C exceeds "
                        f"{self.lim.max_temp} C")
        if setpoint is not None and abs(fb.pos - setpoint) > self.lim.max_track_err:
            raise Abort(f"tracking error {abs(fb.pos - setpoint) / DEG:.1f} deg exceeds "
                        f"{self.lim.max_track_err / DEG:.1f} deg (setpoint {setpoint:+.3f}, "
                        f"pos {fb.pos:+.3f})")


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------

_stop_requested = False


def _on_signal(signum, _frame):
    global _stop_requested
    _stop_requested = True
    print(f"\n[signal {signum}] stopping -- motor will be freed", flush=True)


def _tick_sleep(t_next: float) -> float:
    dt = t_next - time.monotonic()
    if dt > 0:
        time.sleep(dt)
    return t_next


def phase_wake(m: Motor, timeout_s: float = 0.5) -> Feedback:
    """Enter motor mode and read where the motor is BEFORE any gains are applied."""
    m.bus.drain()
    m.enter_motor_mode()
    fb = m.read_feedback(timeout_s)
    if fb is None:
        # Some firmware only answers MIT frames, not the special frames: poke with zero gains.
        m.command(0.0, 0.0, 0.0, 0.0, 0.0)  # kp=kd=t_ff=0 -> 0 N.m regardless of ranges
        fb = m.read_feedback(timeout_s)
    if fb is None:
        raise Abort(f"motor {m.id} did not answer on master id 0x{m.master_id:03X} -- check "
                    f"CAN id, that the drive is in MIT mode, wiring, termination, and that "
                    f"nothing else is holding the bus")
    m.check(fb, None)
    if abs(fb.pos) > m.r.p_max * 0.999:
        raise Abort(f"reported position {fb.pos:+.3f} rad is at the +-{m.r.p_max} rad packing "
                    f"limit -- the drive's P range is probably not what --p-max says")
    return fb


def phase_monitor(m: Motor, rate_hz: float = 10.0) -> None:
    """Zero-gain polling loop: motor stays limp, you turn it by hand and watch the numbers."""
    print("Zero-torque monitor. Turn the shaft by hand: a quarter turn should read ~1.571 rad. "
          "Ctrl-C to stop.")
    period = 1.0 / rate_hz
    t_next = time.monotonic()
    while not _stop_requested:
        m.command(0.0, 0.0, 0.0, 0.0, 0.0)
        fb = m.read_feedback(period * 0.8)
        m.check(fb, None)
        if fb is not None:
            print(f"\r  {fb}", end="", flush=True)
        t_next = _tick_sleep(t_next + period)
    print()


def phase_servo(m: Motor, start: float, end: float, duration_s: float, kp: float, kd: float,
                rate_hz: float, label: str) -> None:
    """Hold (start == end) or linearly ramp the setpoint from start to end over duration_s."""
    period = 1.0 / rate_hz
    n = max(int(round(duration_s * rate_hz)), 1)
    t_next = time.monotonic()
    last_print = 0.0
    for i in range(n + 1):
        if _stop_requested:
            raise Abort("stop requested")
        alpha = i / n
        sp = start + (end - start) * alpha
        m.command(sp, 0.0, kp, kd, 0.0)
        fb = m.read_feedback(period * 0.8)
        m.check(fb, sp)
        now = time.monotonic()
        if fb is not None and (now - last_print > 0.25 or i == n):
            last_print = now
            print(f"\r  [{label}] sp={sp:+.4f} rad ({sp / DEG:+.1f} deg)  {fb}", end="",
                  flush=True)
        t_next = _tick_sleep(t_next + period)
    print()


def phase_free(m: Motor, repeats: int = 3) -> None:
    """Exit motor mode -> zero torque. Sent several times in case a frame is lost."""
    for _ in range(repeats):
        try:
            m.exit_motor_mode()
        except Exception as e:  # noqa: BLE001 - never let a failure here mask the original
            print(f"  warning: exit-motor-mode send failed: {e}")
        time.sleep(0.01)


# ---------------------------------------------------------------------------
# Self-test (no bus)
# ---------------------------------------------------------------------------

def selftest() -> None:
    r = MitRanges()
    # Manual §5.5, "MIT position": pos 2 rad, kp 0.123, kd 0.005 -> 94 7A 7F F0 01 00 47 FF.
    # Our zero-vel / zero-torque codes are 0x800 (manual's upper computer emits 0x7FF); both
    # decode to ~0, so compare the position / kp / kd bits exactly and the rest loosely.
    got = pack_mit(2.0, 0.0, 0.123, 0.005, 0.0, r)
    exp = bytes.fromhex("947A7FF0010047FF")
    assert got[0:2] == exp[0:2], f"position bytes {got[0:2].hex()} != {exp[0:2].hex()}"
    assert (got[3] & 0xF) == (exp[3] & 0xF) and got[4] == exp[4], "kp bits mismatch"
    assert got[5] == exp[5] and (got[6] >> 4) == (exp[6] >> 4), "kd bits mismatch"
    v_code = (got[2] << 4) | (got[3] >> 4)
    t_code = ((got[6] & 0xF) << 8) | got[7]
    assert v_code in (0x7FF, 0x800) and t_code in (0x7FF, 0x800), "zero vel/torque code"
    # Bench capture 2026-09-19 from the GL40 II at id 22: 16 99 21 7F E7 FF 28 00
    fb = unpack_feedback(bytes.fromhex("1699217FE7FF2800"), r)
    assert fb.id_nibble == 0x6 and fb.err == 0x1, fb
    assert abs(fb.pos - 2.454) < 0.002, fb
    assert abs(fb.vel) < 0.2 and abs(fb.torque) < 0.01 and fb.drive_temp == 40, fb
    # Round trip through the drive's own decode of a command
    p_code = (got[0] << 8) | got[1]
    assert abs(uint_to_float(p_code, -r.p_max, r.p_max, 16) - 2.0) < 1e-3
    # Gain quantisation: main() snaps to the nearest count and nudges +0.5 count before the
    # drive-side truncation, so 0.61 -> raw 5 (not the truncated raw 4).
    kp_step, kd_step = r.kp_max / 4096, r.kd_max / 4096
    assert float_to_uint((round(0.61 / kp_step) + 0.5) * kp_step, 0, r.kp_max, 12) == 5
    assert float_to_uint((round(0.366 / kp_step) + 0.5) * kp_step, 0, r.kp_max, 12) == 3
    assert float_to_uint((round(0.0098 / kd_step) + 0.5) * kd_step, 0, r.kd_max, 12) == 8
    print("selftest OK: packing matches GL II manual §5.5 example and bench feedback capture")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--id", type=int, default=22, help="motor CAN node id (default 22 = wrist GL40)")
    ap.add_argument("--master-id", type=lambda s: int(s, 0), default=0,
                    help="feedback CAN id set in the drive (default 0)")
    ap.add_argument("--iface", default="can0")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--deg", type=float, help="target in degrees")
    mode.add_argument("--rad", type=float, help="target in radians")
    mode.add_argument("--monitor", action="store_true",
                      help="zero-torque loop printing decoded feedback; verifies --p-max scaling")
    mode.add_argument("--selftest", action="store_true", help="check frame packing, no bus")
    ap.add_argument("--absolute", action="store_true",
                    help="target is absolute (drive zero); default is relative to current position")
    ap.add_argument("--kp", type=float, default=0.366,
                    help="N.m/rad, snapped to the nearest 500/4096 count (default raw 3 = 0.366); "
                         "kp * --max-track-err must stay under --max-torque")
    ap.add_argument("--kd", type=float, default=0.0098,
                    help="N.m.s/rad, snapped to the nearest 5/4096 count; must be > 0 (default raw 8)")
    ap.add_argument("--duration", type=float, default=4.0, help="ramp time, s (default 4)")
    ap.add_argument("--rate", type=float, default=50.0, help="command rate, Hz (default 50)")
    ap.add_argument("--hold", action="store_true",
                    help="keep holding at the target until Ctrl-C instead of freeing the motor")
    ap.add_argument("--p-max", type=float, default=12.5, help="drive P range, rad (default 12.5)")
    ap.add_argument("--v-max", type=float, default=200.0, help="drive V range (default 200)")
    ap.add_argument("--t-max", type=float, default=10.0, help="drive T range, N.m (default 10)")
    ap.add_argument("--max-torque", type=float, default=0.3,
                    help="N.m: refuse kp*|error| above this and abort on feedback above it "
                         "(default 0.3 -- GL40 rated 0.25, peak 0.73)")
    ap.add_argument("--max-track-err", type=float, default=15.0,
                    help="deg: abort if the motor lags the setpoint by more (default 15)")
    ap.add_argument("--max-temp", type=int, default=60, help="degC abort threshold (default 60)")
    ap.add_argument("--max-setpoint-vel", type=float, default=1.0,
                    help="rad/s: --duration is stretched so the ramp never exceeds this (default 1)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the frames instead of sending them (no feedback -> no move)")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        selftest()
        return 0
    if not (args.monitor or args.deg is not None or args.rad is not None):
        build_parser().error("one of --deg / --rad / --monitor / --selftest is required")

    ranges = MitRanges(p_max=args.p_max, v_max=args.v_max, t_max=args.t_max)
    limits = SafetyLimits(max_torque=args.max_torque, max_track_err=args.max_track_err * DEG,
                          max_temp=args.max_temp, feedback_timeout=0.2,
                          max_setpoint_vel=args.max_setpoint_vel)

    if not args.monitor:
        if args.kd <= 0.0:
            sys.exit("--kd must be > 0: the GL II manual warns kd=0 in position control "
                     "makes the motor oscillate / run away")
        if args.kp <= 0.0:
            sys.exit("--kp must be > 0 for a position move (use --monitor for zero-torque)")
        # Snap to the nearest 12-bit code (the drive truncates, so 0.61 would otherwise become
        # raw 4 = 0.488). kp_q/kd_q are what the drive applies; kp_send/kd_send are nudged half a
        # count up so float_to_uint's truncation lands exactly on that code.
        kp_step, kd_step = ranges.kp_max / 4096, ranges.kd_max / 4096
        kp_raw, kd_raw = round(args.kp / kp_step), round(args.kd / kd_step)
        if kp_raw < 1 or kd_raw < 1:
            sys.exit(f"gains quantise to zero (kp step {kp_step:.4f}, kd step {kd_step:.5f}) "
                     f"-- raise --kp/--kd")
        kp_q, kd_q = kp_raw * kp_step, kd_raw * kd_step
        kp_send, kd_send = (kp_raw + 0.5) * kp_step, (kd_raw + 0.5) * kd_step
        print(f"gains as the drive will see them: kp={kp_q:.4f} N.m/rad (raw {kp_raw})  "
              f"kd={kd_q:.5f} N.m.s/rad (raw {kd_raw})")
        # Worst case is a stalled motor: the tracking-error abort fires at max_track_err, so the
        # PD torque can never exceed kp * max_track_err (+ kd * vel, negligible at ramp speeds).
        worst_torque = kp_q * limits.max_track_err
        if worst_torque > limits.max_torque:
            sys.exit(f"kp {kp_q:.3f} x --max-track-err {limits.max_track_err / DEG:.0f} deg = "
                     f"{worst_torque:.3f} N.m would exceed --max-torque {limits.max_torque}; "
                     f"lower --kp or --max-track-err")

    print("\n  HARDWARE E-STOP: make sure a physical cutoff on the motor supply is within reach.\n"
          "  Software exit-motor-mode (sent on Ctrl-C / any abort) is a complement, not a "
          "replacement.\n")

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    bus = CanBus(args.iface, dry_run=args.dry_run)
    m = Motor(bus, args.id, args.master_id, ranges, limits)
    rc = 0
    try:
        if args.dry_run:
            # No feedback in dry-run: show the frames using the bench position as an example.
            cur = 2.454
            print(f"[dry-run] pretending current position = {cur:+.3f} rad")
            print("[dry-run] enter motor mode:")
            m.enter_motor_mode()
        else:
            print(f"Waking motor {args.id} on {args.iface} (feedback id 0x{args.master_id:03X})...")
            fb0 = phase_wake(m)
            print(f"  {fb0}")
            cur = fb0.pos

        if args.monitor:
            if args.dry_run:
                print("[dry-run] monitor frame (zero gains):")
                m.command(0.0, 0.0, 0.0, 0.0, 0.0)
            else:
                phase_monitor(m)
            return 0

        target = args.rad if args.rad is not None else args.deg * DEG
        if not args.absolute:
            target = cur + target
        if abs(target) > ranges.p_max:
            raise Abort(f"target {target:+.3f} rad is outside the drive's +-{ranges.p_max} rad")
        delta = target - cur
        duration = max(args.duration, abs(delta) / limits.max_setpoint_vel)
        print(f"Move: {cur:+.4f} -> {target:+.4f} rad (delta {delta / DEG:+.1f} deg) over "
              f"{duration:.1f} s at {args.rate:.0f} Hz; worst-case (stalled) PD torque "
              f"{worst_torque:.3f} N.m")

        if args.dry_run:
            print("[dry-run] hold frame:")
            m.command(cur, 0.0, kp_send, kd_send, 0.0)
            print("[dry-run] first / last ramp frame:")
            m.command(cur + delta / max(duration * args.rate, 1), 0.0, kp_send, kd_send, 0.0)
            m.command(target, 0.0, kp_send, kd_send, 0.0)
            print("[dry-run] exit motor mode:")
            phase_free(m, repeats=1)
            return 0

        phase_servo(m, cur, cur, 1.0, kp_send, kd_send, args.rate, "hold")
        phase_servo(m, cur, target, duration, kp_send, kd_send, args.rate, "ramp")
        phase_servo(m, target, target, 2.0, kp_send, kd_send, args.rate, "settle")
        fb = m.last_fb
        if fb is not None:
            print(f"Reached: pos={fb.pos:+.4f} rad ({fb.pos / DEG:+.1f} deg), "
                  f"error {(fb.pos - target) / DEG:+.2f} deg")
        if args.hold:
            print("Holding at target. Ctrl-C to free the motor.")
            while not _stop_requested:
                phase_servo(m, target, target, 1.0, kp_send, kd_send, args.rate, "hold")
        return 0
    except Abort as e:
        print(f"\nABORT: {e}")
        rc = 2
    except Exception as e:  # noqa: BLE001
        print(f"\nERROR: {type(e).__name__}: {e}")
        rc = 3
    finally:
        # Always leave the motor limp -- including after --hold (Ctrl-C lands here too).
        if not args.dry_run:
            print("Freeing motor (exit motor mode)...")
            phase_free(m)
        bus.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
