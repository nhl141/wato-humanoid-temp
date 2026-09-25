#!/usr/bin/env python3
"""Emulate the arm's seven CubeMars drives on a virtual CAN bus -- no hardware, no risk.

Speaks both dialects the real arm uses, so can_node / joint_command / the bench scripts can be
exercised end to end before anything is plugged in:

  ids 10-14  AK10-9 / AK80-9 in SERVO mode  -- extended frames (0x400|id position loop,
             0xF00|id disable, ...), ServoStatusFeedback on 0x2900|id, streamed at
             --feedback-hz like a configured drive.
             With --ak-mode mit they instead emulate MIT mode (AK manual V3.2.0 section 4.2):
             standard frames on the node id, feedback on the master id with the FULL id in
             byte 0, a gravity load, and the drive's CAN timeout (--ak-can-timeout): if
             commands stop, output is cut and the joint falls -- which is what the damp fault
             action in joint_command exists to prevent.
  ids 21,22  GL40 KV70 on a GL II drive in MIT mode -- standard 11-bit frames on the node id,
             feedback on the master id (0x000), and ONLY when spoken to (exactly like the real
             drive: it answers a frame, it never volunteers). Ignores MIT commands until it
             has been sent "enter motor mode" (FF..FC), goes limp on FF..FD.

Each motor runs a small plant: inertia, damping, and a gravity-like restoring load, so a PD
hold sags under load the way the real wrist did on the bench (kp 1.22 held +34 of a commanded
+40 deg at 0.125 N.m). That is what makes gain and clamp benchmarks meaningful here.

Setup (host, once per boot -- vcan is a kernel module, nothing to do with the CANable)::

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan 2>/dev/null || true
    sudo ip link set up vcan0

Then, with the containers on host networking, vcan0 is visible inside them too::

    python3 gl40_sim.py --iface vcan0                  # leave running
    ros2 launch can can.launch.py --ros-args -p can_interface:=vcan0 -p bustype:=socketcan
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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

DEG = math.pi / 180.0

_CAN_FRAME_FMT = "=IB3x8s"
_CAN_FRAME_SIZE = struct.calcsize(_CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF

# Servo-mode command ids (base | motor id), from dbc/humanoid.dbc.
SERVO_DUTY = 0x000
SERVO_CURRENT = 0x100
SERVO_BRAKE = 0x200
SERVO_VELOCITY = 0x300
SERVO_POSITION = 0x400
SERVO_SET_ORIGIN = 0x500
SERVO_POS_VEL = 0x600
SERVO_DISABLE = 0xF00
SERVO_FEEDBACK = 0x2900

MIT_ENTER, MIT_EXIT, MIT_SET_ZERO, MIT_CLEAR = 0xFC, 0xFD, 0xFE, 0xFB


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    x = min(max(x, x_min), x_max)
    return min(int((x - x_min) * ((1 << bits) / span)), (1 << bits) - 1)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    return float(x_int) * (x_max - x_min) / float((1 << bits) - 1) + x_min


@dataclass
class Plant:
    """Shared mechanics: inertia + damping + a gravity-like restoring load."""

    pos: float = 0.0      # rad
    vel: float = 0.0      # rad/s
    inertia: float = 0.002
    damping: float = 0.02
    load_gain: float = 0.15   # N.m of restoring torque at 90 deg from rest
    rest: float = 0.0         # rad the shaft falls back to when unpowered
    temp_c: int = 40

    def step(self, torque: float, dt: float, vel_limit: float = 50.0) -> None:
        load = -self.load_gain * math.sin(self.pos - self.rest)
        acc = (torque + load - self.damping * self.vel) / self.inertia
        self.vel = max(-vel_limit, min(vel_limit, self.vel + acc * dt))
        self.pos += self.vel * dt


@dataclass
class AkMotor:
    """AK-series drive in servo mode: an internal position loop we approximate as a rate limit."""

    motor_id: int
    model: str
    max_vel_dps: float = 60.0
    plant: Plant = field(default_factory=lambda: Plant(load_gain=0.0, damping=0.5))
    target_deg: Optional[float] = None
    enabled: bool = True
    current_a: float = 0.0

    def step(self, dt: float) -> None:
        if not self.enabled or self.target_deg is None:
            self.plant.vel = 0.0
            self.current_a = 0.0
            return
        pos_deg = self.plant.pos / DEG
        err = self.target_deg - pos_deg
        step_deg = max(-self.max_vel_dps * dt, min(self.max_vel_dps * dt, err))
        self.plant.pos = (pos_deg + step_deg) * DEG
        self.plant.vel = (step_deg / dt) * DEG
        self.current_a = max(-60.0, min(60.0, err * 0.1))

    def feedback_frame(self):
        pos_deg = self.plant.pos / DEG
        erpm = self.plant.vel / DEG / 6.0 * 10.0  # deg/s -> rpm -> ERPM-ish, scale 10 in the DBC
        data = struct.pack(
            ">hhhbB",
            max(-32768, min(32767, int(round(pos_deg * 10)))),
            max(-32768, min(32767, int(round(erpm / 10)))),
            max(-32768, min(32767, int(round(self.current_a * 100)))),
            self.plant.temp_c,
            0,  # error code: no fault
        )
        return (CAN_EFF_FLAG | SERVO_FEEDBACK | self.motor_id, data)


@dataclass
class Gl2Motor:
    """GL II drive in MIT mode: PD torque from the last command, answers only when spoken to."""

    motor_id: int
    model: str = "GL40-KV70"
    p_max: float = 12.5
    v_max: float = 200.0
    t_max: float = 10.0
    kp_max: float = 500.0
    kd_max: float = 5.0
    torque_limit: float = 0.73  # GL40 KV70 peak
    plant: Plant = field(default_factory=lambda: Plant(load_gain=0.15))
    entered: bool = False
    status: int = 0  # 0 = Disable, 1 = Enable
    last_cmd: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)
    torque: float = 0.0

    def handle_command(self, data: bytes) -> None:
        if len(data) >= 8 and data[:7] == b"\xff" * 7:
            code = data[7]
            if code == MIT_ENTER:
                self.entered, self.status = True, 1
            elif code == MIT_EXIT:
                self.entered, self.status = False, 0
                self.last_cmd = (0.0, 0.0, 0.0, 0.0, 0.0)
            elif code == MIT_SET_ZERO:
                self.plant.rest -= self.plant.pos
                self.plant.pos = 0.0
            elif code == MIT_CLEAR:
                self.status = 1 if self.entered else 0
            return

        if len(data) < 8:
            return
        p_i = (data[0] << 8) | data[1]
        v_i = (data[2] << 4) | (data[3] >> 4)
        kp_i = ((data[3] & 0xF) << 8) | data[4]
        kd_i = (data[5] << 4) | (data[6] >> 4)
        t_i = ((data[6] & 0xF) << 8) | data[7]
        self.last_cmd = (
            uint_to_float(p_i, -self.p_max, self.p_max, 16),
            uint_to_float(v_i, -self.v_max, self.v_max, 12),
            uint_to_float(kp_i, 0.0, self.kp_max, 12),
            uint_to_float(kd_i, 0.0, self.kd_max, 12),
            uint_to_float(t_i, -self.t_max, self.t_max, 12),
        )

    def step(self, dt: float) -> None:
        if self.entered:
            p_des, v_des, kp, kd, t_ff = self.last_cmd
            tau = kp * (p_des - self.plant.pos) + kd * (v_des - self.plant.vel) + t_ff
            self.torque = max(-self.torque_limit, min(self.torque_limit, tau))
        else:
            self.torque = 0.0  # limp: only the load acts
        self.plant.step(self.torque, dt)

    def feedback_frame(self, master_id: int):
        p_i = float_to_uint(self.plant.pos, -self.p_max, self.p_max, 16)
        v_i = float_to_uint(self.plant.vel, -self.v_max, self.v_max, 12)
        t_i = float_to_uint(self.torque, -self.t_max, self.t_max, 12)
        data = bytes([
            ((self.status & 0xF) << 4) | (self.motor_id & 0xF),
            (p_i >> 8) & 0xFF, p_i & 0xFF,
            (v_i >> 4) & 0xFF,
            ((v_i & 0xF) << 4) | ((t_i >> 8) & 0xF),
            t_i & 0xFF,
            self.plant.temp_c & 0xFF,
            0,
        ])
        return (master_id, data)


@dataclass
class AkMitMotor(Gl2Motor):
    """AK drive in MIT mode: same PD/command frame as GL II, AK feedback layout, CAN timeout.

    The feedback layout follows the manual, not a bench capture -- the same caveat as
    decodeAkFeedback() in mit_protocol.cpp.
    """

    p_max: float = 12.56
    v_max: float = 65.0
    t_max: float = 18.0
    torque_limit: float = 22.0
    can_timeout: float = 0.2      # s without a command before the drive cuts output
    last_rx: float = 0.0
    timed_out: bool = False

    def handle_command(self, data: bytes) -> None:
        self.last_rx = time.monotonic()
        self.timed_out = False
        super().handle_command(data)

    def step(self, dt: float) -> None:
        if self.entered and self.can_timeout > 0 and \
                time.monotonic() - self.last_rx > self.can_timeout:
            self.timed_out = True
        if self.timed_out:
            self.torque = 0.0  # drive cut output: only the load acts
            self.plant.step(0.0, dt)
            return
        super().step(dt)

    def feedback_frame(self, master_id: int):
        p_i = float_to_uint(self.plant.pos, -self.p_max, self.p_max, 16)
        v_i = float_to_uint(self.plant.vel, -self.v_max, self.v_max, 12)
        t_i = float_to_uint(self.torque, -self.t_max, self.t_max, 12)
        data = bytes([
            self.motor_id & 0xFF,
            (p_i >> 8) & 0xFF, p_i & 0xFF,
            (v_i >> 4) & 0xFF,
            ((v_i & 0xF) << 4) | ((t_i >> 8) & 0xF),
            t_i & 0xFF,
            self.plant.temp_c & 0xFF,
            0,  # error: none
        ])
        return (master_id, data)


# Rough gravity load per AK joint in MIT mode (N.m at 90 deg from rest) and peak torque.
_AK_MIT_PLANT = {
    14: ("AK10-9", 5.0, 53.0), 12: ("AK10-9", 3.0, 53.0),
    11: ("AK80-9", 0.5, 22.0), 10: ("AK80-9", 2.0, 22.0), 13: ("AK80-9", 0.3, 22.0),
}


class Simulator:
    def __init__(self, iface: str, master_id: int, feedback_hz: float, load_gain: float,
                 quiet: bool, gl_start_deg: float = 45.0, ak_start_cmd_deg: float = 0.0,
                 calibration: Optional[Dict[int, tuple]] = None, ak_mode: str = "servo",
                 ak_can_timeout: float = 0.2):
        calibration = calibration or {}
        self.iface = iface
        self.master_id = master_id
        self.feedback_period = 1.0 / feedback_hz if feedback_hz > 0 else None
        self.quiet = quiet
        self.stop = False

        # Matches joint_command/config/hardware_mapping.yaml (left arm).
        self.ak: Dict[int, AkMotor] = {
            14: AkMotor(14, "AK10-9"), 12: AkMotor(12, "AK10-9"),
            11: AkMotor(11, "AK80-9"), 10: AkMotor(10, "AK80-9"), 13: AkMotor(13, "AK80-9"),
        }
        # Park every AK at a given COMMAND-frame angle, converted to the motor frame the drive
        # actually reports: motor = direction * (cmd - zero_offset). Feeding back +zero_offset
        # instead (the obvious-looking choice) puts every joint hundreds of degrees outside its
        # own limits, which joint_command correctly refuses to command.
        for motor_id, motor in self.ak.items():
            cal = calibration.get(motor_id)
            if cal is None:
                continue
            direction, zero_offset = cal
            motor_deg = direction * (ak_start_cmd_deg - zero_offset)
            motor.plant.pos = motor_deg * DEG
            motor.target_deg = motor_deg
        # MIT mode: the same joints, now PD drives under a gravity load that rests where the
        # joint was parked (an unpowered arm hangs there) -- so once commanded away, a dropped
        # joint visibly falls back.
        self.ak_mit: Dict[int, AkMitMotor] = {}
        if ak_mode == "mit":
            for motor_id, motor in self.ak.items():
                model, load, peak = _AK_MIT_PLANT[motor_id]
                start = motor.plant.pos
                self.ak_mit[motor_id] = AkMitMotor(
                    motor_id, model=model, torque_limit=peak, can_timeout=ak_can_timeout,
                    plant=Plant(pos=start, rest=start, load_gain=load,
                                inertia=0.05, damping=0.2))
            self.ak = {}
        # The real bench wrist sits near +140 deg on its own scale, which is OUTSIDE the
        # placeholder +-90 limits in hardware_mapping.yaml -- useful for exercising the
        # "joint is outside its own limits" exclusion, but it blocks the joint, so the
        # default start is inside the limits. Pass --gl-start-deg 143.5 for the real case.
        self.gl: Dict[int, Gl2Motor] = {
            22: Gl2Motor(22, plant=Plant(load_gain=load_gain,
                                         rest=(gl_start_deg - 3.5) * DEG,
                                         pos=gl_start_deg * DEG)),
            21: Gl2Motor(21, plant=Plant(load_gain=load_gain * 0.5, rest=0.0, pos=5.0 * DEG)),
        }

        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)
        self.rx = 0
        self.tx = 0

    def send(self, can_id: int, data: bytes) -> None:
        self.sock.send(struct.pack(_CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b"\0")))
        self.tx += 1

    def log(self, msg: str) -> None:
        if not self.quiet:
            print(msg, flush=True)

    def handle_frame(self, can_id: int, data: bytes) -> None:
        self.rx += 1
        if can_id & CAN_EFF_FLAG:
            raw = can_id & CAN_EFF_MASK
            motor_id, base = raw & 0xFF, raw & 0xFFFFFF00
            motor = self.ak.get(motor_id)
            if motor is None:
                return
            if base == SERVO_POSITION and len(data) >= 4:
                motor.target_deg = struct.unpack(">i", data[:4])[0] / 10000.0
                motor.enabled = True
            elif base == SERVO_DISABLE:
                motor.enabled = False
                self.log(f"  [sim] AK {motor_id} disabled (limp)")
            elif base == SERVO_SET_ORIGIN:
                motor.plant.pos = 0.0
                motor.target_deg = 0.0
                self.log(f"  [sim] AK {motor_id} origin set")
            elif base == SERVO_VELOCITY and len(data) >= 4:
                motor.target_deg = motor.plant.pos / DEG  # not modelled; hold
            return

        # Standard frame: a MIT command (GL II, or AK in --ak-mode mit) addressed by node id.
        node = can_id & CAN_SFF_MASK
        motor = self.gl.get(node) or self.ak_mit.get(node)
        if motor is None:
            return
        was_entered = motor.entered
        motor.handle_command(bytes(data))
        if motor.entered != was_entered:
            self.log(f"  [sim] {motor.model} {node} "
                     f"{'ENTERED motor mode' if motor.entered else 'freed'}")
        # The real drive answers every frame it accepts -- including the special frames.
        can_id_fb, payload = motor.feedback_frame(self.master_id)
        self.send(can_id_fb, payload)

    def run(self, rate_hz: float = 1000.0) -> None:
        dt = 1.0 / rate_hz
        next_fb = time.monotonic()
        next_status = time.monotonic() + 5.0
        ak_ids = sorted(self.ak) or sorted(self.ak_mit)
        ak_mode = "servo" if self.ak else "MIT"
        self.log(f"GL II + AK simulator on {self.iface}: AK ids {ak_ids} ({ak_mode}), "
                 f"GL40 ids {sorted(self.gl)} (MIT, master id 0x{self.master_id:03X}). Ctrl-C to stop.")
        while not self.stop:
            now = time.monotonic()
            # Drain everything queued.
            while True:
                try:
                    raw = self.sock.recv(_CAN_FRAME_SIZE)
                except BlockingIOError:
                    break
                can_id, dlc, data = struct.unpack(_CAN_FRAME_FMT, raw)
                if can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG):
                    continue
                self.handle_frame(can_id, data[:dlc])

            for motor in self.ak.values():
                motor.step(dt)
            for motor in self.gl.values():
                motor.step(dt)
            for motor in self.ak_mit.values():
                was_timed_out = motor.timed_out
                motor.step(dt)
                if motor.timed_out and not was_timed_out:
                    self.log(f"  [sim] AK {motor.motor_id} CAN TIMEOUT -- output cut, joint falls")

            # AK drives stream status; GL II drives never volunteer one.
            if self.feedback_period is not None and now >= next_fb:
                next_fb = now + self.feedback_period
                for motor in self.ak.values():
                    can_id, payload = motor.feedback_frame()
                    self.send(can_id, payload)

            if not self.quiet and now >= next_status:
                next_status = now + 5.0
                gl = "  ".join(
                    f"GL{m.motor_id}={m.plant.pos / DEG:+.1f}deg tau={m.torque:+.3f}"
                    f"{'' if m.entered else ' (limp)'}" for m in self.gl.values())
                ak = "  ".join(f"AK{m.motor_id}={m.plant.pos / DEG:+.1f}deg"
                               for m in self.ak.values())
                ak += "  ".join(
                    f"AK{m.motor_id}={m.plant.pos / DEG:+.1f}deg tau={m.torque:+.2f}"
                    f"{' (TIMEOUT)' if m.timed_out else '' if m.entered else ' (limp)'}"
                    for m in self.ak_mit.values())
                print(f"  [sim] rx={self.rx} tx={self.tx} | {ak} | {gl}", flush=True)

            time.sleep(max(0.0, dt - (time.monotonic() - now)))


def load_calibration(args) -> Dict[int, tuple]:
    """{motor_id: (direction, zero_offset)} from hardware_mapping.yaml, or {} if unavailable."""
    candidates = [args.mapping] if args.mapping else [
        "/calibration/hardware_mapping.yaml",
        str(Path(__file__).resolve().parents[2] / "joint_command/config/hardware_mapping.yaml"),
    ]
    for path in candidates:
        if not path or not Path(path).exists():
            continue
        try:
            import yaml
        except ImportError:
            print("  [sim] PyYAML missing: AK joints start at motor-frame 0")
            return {}
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")).get(args.arm_side, {})
        out: Dict[int, tuple] = {}

        def walk(node):
            if not isinstance(node, dict):
                return
            if "can_id" in node:
                out[int(node["can_id"])] = (float(node.get("direction", 1) or 1),
                                            float(node.get("zero_offset", 0.0)))
                return
            for child in node.values():
                walk(child)

        walk(data)
        print(f"  [sim] calibration from {path}")
        return out
    return {}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--iface", default="vcan0", help="CAN interface (default vcan0)")
    ap.add_argument("--master-id", type=lambda s: int(s, 0), default=0,
                    help="id the GL II drives reply on (default 0)")
    ap.add_argument("--feedback-hz", type=float, default=50.0,
                    help="AK servo status rate; 0 disables (default 50)")
    ap.add_argument("--load-gain", type=float, default=0.15,
                    help="N.m of gravity-like restoring torque on the GL40s at 90 deg from "
                         "rest (default 0.15, matching the bench wrist)")
    ap.add_argument("--rate-hz", type=float, default=1000.0, help="plant integration rate")
    ap.add_argument("--gl-start-deg", type=float, default=45.0,
                    help="where the GL40 wrist (id 22) starts, in its own drive frame "
                         "(default 45, inside hardware_mapping.yaml's placeholder limits; the "
                         "real bench wrist reads ~143.5, which those limits exclude)")
    ap.add_argument("--ak-start-cmd-deg", type=float, default=0.0,
                    help="COMMAND-frame angle the five AK joints start at (default 0); "
                         "converted to the motor frame with hardware_mapping.yaml")
    ap.add_argument("--mapping", help="hardware_mapping.yaml used for that conversion")
    ap.add_argument("--ak-mode", choices=("servo", "mit"), default="servo",
                    help="protocol the five AK drives speak (default servo)")
    ap.add_argument("--ak-can-timeout", type=float, default=0.2,
                    help="MIT mode: s without a command before an AK cuts output; 0 disables "
                         "(default 0.2, the R-Link setting to use on the real drives)")
    ap.add_argument("--arm-side", default="left")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    if not os.path.exists(f"/sys/class/net/{args.iface}"):
        sys.exit(f"interface {args.iface} does not exist. Create it with:\n"
                 f"  sudo modprobe vcan\n"
                 f"  sudo ip link add dev {args.iface} type vcan\n"
                 f"  sudo ip link set up {args.iface}")

    sim = Simulator(args.iface, args.master_id, args.feedback_hz, args.load_gain, args.quiet,
                    args.gl_start_deg, args.ak_start_cmd_deg, load_calibration(args),
                    args.ak_mode, args.ak_can_timeout)

    def on_signal(signum, _frame):
        sim.stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    sim.run(args.rate_hz)
    print(f"\nsimulator stopped (rx={sim.rx} tx={sim.tx})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
