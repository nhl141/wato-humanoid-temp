#!/usr/bin/env python3
"""Step-response and gain-sweep benchmarks for ONE CubeMars GL II drive (GL40 II) in MIT mode.

Builds on gl40_mit_move.py (same wake / hold / free scaffolding, same safety net) and adds the
measurements you need to choose stiffness gains:

  --step DEG          hold, apply a STEP setpoint (no ramp), hold, ramp back, free
  --sweep "kp,kp,..." one --step run per gain, printing rise time / overshoot / sag / peak torque
  --report RUNDIR     offline: recompute the numbers from a run folder (no bus, no motor)

A step is the only way to see stiffness: a slow ramp keeps the tracking error -- and therefore
the PD torque -- near zero, so every gain looks the same. That makes a step inherently more
aggressive, so it is bounded twice: the requested step must satisfy kp*step <= --max-torque AND
step < --max-track-err, i.e. the motor can neither pull harder than the ceiling nor run away
before the tracking abort fires.

Examples (inside the interfacing container; can0 up; joint_command_node NOT running)::

  S=/root/ament_ws/src/interfacing/can/scripts
  sudo python3 $S/gl40_bench.py --id 22 --step 5 --kp 1.22 --max-track-err 12
  sudo python3 $S/gl40_bench.py --id 22 --step 5 --sweep "0.61,0.85,1.22,1.34" --max-track-err 12
  python3 $S/gl40_bench.py --report outputs/gl40_bench/20260921-174455_id22-step5
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import List, Optional

import gl40_mit_move as g
import telemetry
from telemetry import RunFolder

DEG = g.DEG


def parse_gain_list(text: Optional[str]) -> List[float]:
    if not text:
        return []
    return [float(chunk) for chunk in text.replace(" ", "").split(",") if chunk]


def snap_gains(kp: float, kd: float, ranges: g.MitRanges):
    """Nearest 12-bit codes + the values to pack (see gl40_mit_move.main)."""
    kp_step, kd_step = ranges.kp_max / 4096, ranges.kd_max / 4096
    kp_raw, kd_raw = round(kp / kp_step), round(kd / kd_step)
    if kp_raw < 1 or kd_raw < 1:
        raise SystemExit(f"gains quantise to zero (kp step {kp_step:.4f}, kd step "
                         f"{kd_step:.5f}) -- raise --kp/--kd")
    return (kp_raw, kd_raw, kp_raw * kp_step, kd_raw * kd_step,
            (kp_raw + 0.5) * kp_step, (kd_raw + 0.5) * kd_step)


def run_step(motor: g.Motor, start: float, step_rad: float, kp_send: float, kd_send: float,
             rate: float, hold_s: float, settle_s: float, return_s: float) -> None:
    """hold at start -> STEP to start+step -> settle -> ramp back -> (caller frees)."""
    target = start + step_rad
    g.phase_servo(motor, start, start, hold_s, kp_send, kd_send, rate, "hold")
    # The step itself: phase_servo with start == end jumps the setpoint immediately.
    g.phase_servo(motor, target, target, settle_s, kp_send, kd_send, rate, "step")
    # Ramp back gently rather than stepping again -- an unloaded shaft snapping back is what
    # chews mounts.
    g.phase_servo(motor, target, start, return_s, kp_send, kd_send, rate, "ramp")


def bench_once(args, kp: float, kd: float, label_suffix: str = "") -> int:
    ranges = g.MitRanges(p_max=args.p_max, v_max=args.v_max, t_max=args.t_max)
    try:
        soft_lo, soft_hi = g.parse_soft_limits(args.soft_limits)
    except ValueError as e:
        sys.exit(str(e))

    kp_raw, kd_raw, kp_q, kd_q, kp_send, kd_send = snap_gains(kp, kd, ranges)
    step_rad = args.step * DEG
    limits = g.SafetyLimits(
        max_torque=args.max_torque, max_track_err=args.max_track_err * DEG,
        max_temp=args.max_temp, feedback_timeout=0.2,
        max_setpoint_vel=args.max_setpoint_vel, max_shaft_vel=args.max_shaft_vel,
        soft_lo=soft_lo, soft_hi=soft_hi, soft_margin=args.soft_limit_margin * DEG)

    # Two independent bounds on how hard this step can pull.
    worst = kp_q * abs(step_rad)
    if worst > limits.max_torque:
        sys.exit(f"kp {kp_q:.3f} N.m/rad x step {args.step:.1f} deg = {worst:.3f} N.m exceeds "
                 f"--max-torque {limits.max_torque} N.m -- lower --kp or --step")
    if abs(step_rad) >= limits.max_track_err:
        sys.exit(f"step {args.step:.1f} deg is at/above the tracking abort "
                 f"({args.max_track_err:.1f} deg): the step would abort instantly. "
                 f"Use a smaller --step or a larger --max-track-err.")

    print(f"\ngains as the drive will see them: kp={kp_q:.4f} N.m/rad (raw {kp_raw})  "
          f"kd={kd_q:.5f} N.m.s/rad (raw {kd_raw});  worst-case step torque {worst:.3f} N.m")
    print("\n  HARDWARE E-STOP: physical cutoff on the motor supply within reach.\n")

    # Open the bus before creating the run folder (CanBus exits when the interface is absent).
    bus = g.CanBus(args.iface, dry_run=False)

    label = args.label or f"id{args.id}-step{args.step:+.0f}deg-kp{kp_q:.2f}{label_suffix}"
    log = RunFolder(
        label=label.replace("+", "p").replace("-", "m"),
        source="script",
        enabled=not args.no_log,
        meta={
            "tool": "gl40_bench.py", "motor_id": args.id, "joint": args.joint,
            "mode": "step", "step_deg": args.step, "rate_hz": args.rate,
            "ranges": {"p_max": args.p_max, "v_max": args.v_max, "t_max": args.t_max},
            "limits": {
                "max_torque_nm": args.max_torque, "max_track_err_deg": args.max_track_err,
                "max_temp_c": args.max_temp, "max_shaft_vel_rad_s": args.max_shaft_vel,
                "max_setpoint_vel_rad_s": args.max_setpoint_vel,
                "soft_limits_deg": None if soft_lo is None and soft_hi is None else [
                    None if soft_lo is None else round(soft_lo / DEG, 3),
                    None if soft_hi is None else round(soft_hi / DEG, 3)],
                "soft_limit_margin_deg": args.soft_limit_margin,
                "feedback_timeout_s": 0.2,
            },
            "gains": {"kp_requested": kp, "kd_requested": kd, "kp_applied": kp_q,
                      "kd_applied": kd_q, "kp_raw": kp_raw, "kd_raw": kd_raw},
            "velocity_scale_verified": False,
        },
    )
    if log.enabled:
        print(log.describe())

    m = g.Motor(bus, args.id, args.master_id, ranges, limits, log=log, joint=args.joint)
    rc = 0
    try:
        print(f"Waking motor {args.id} on {args.iface}...")
        fb0 = g.phase_wake(m)
        print(f"  {fb0}")
        start = fb0.pos
        target = start + step_rad
        if g.clamp_setpoint(target, soft_lo, soft_hi) != target:
            raise g.Abort(f"step target {target / DEG:+.1f} deg is outside --soft-limits")
        if abs(target) > ranges.p_max:
            raise g.Abort(f"step target {target:+.3f} rad is outside +-{ranges.p_max} rad")
        print(f"Step: {start / DEG:+.1f} -> {target / DEG:+.1f} deg "
              f"({args.step:+.1f} deg), settle {args.settle:.1f}s, return {args.return_time:.1f}s")
        run_step(m, start, step_rad, kp_send, kd_send, args.rate, args.hold, args.settle,
                 args.return_time)
    except g.Abort as e:
        print(f"\nABORT: {e}")
        log.note(outcome="aborted", abort_reason=str(e))
        rc = 2
    except Exception as e:  # noqa: BLE001
        print(f"\nERROR: {type(e).__name__}: {e}")
        log.note(outcome="error", abort_reason=f"{type(e).__name__}: {e}")
        rc = 3
    finally:
        print("Freeing motor (exit motor mode)...")
        g.phase_free(m)
        bus.close()
        run_dir = log.close(log.meta.get("outcome", "completed"))

    if run_dir is not None:
        meta, rows = telemetry.load_run(run_dir)
        per_motor = telemetry.run_metrics(meta, rows)
        summary = telemetry.format_metrics(meta, per_motor)
        (run_dir / "summary.md").write_text(summary, encoding="utf-8")
        print()
        print(summary)
        print(f"Telemetry: {run_dir}")
    return rc


def cmd_report(args) -> int:
    run_dir = Path(args.report)
    if not (run_dir / "telemetry.csv").exists():
        sys.exit(f"{run_dir} has no telemetry.csv -- is it a run folder?")
    meta, rows = telemetry.load_run(run_dir)
    per_motor = telemetry.run_metrics(meta, rows)
    summary = telemetry.format_metrics(meta, per_motor)
    (run_dir / "summary.md").write_text(summary, encoding="utf-8")
    print(summary)
    if args.json:
        print(json.dumps(per_motor, indent=2, sort_keys=True))
    failed = [mid for mid, m in per_motor.items() if m.get("passed") is False]
    if failed:
        print(f"FAILED checks for motor(s): {failed}")
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--id", type=int, default=22, help="motor CAN node id (default 22 = wrist)")
    ap.add_argument("--master-id", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--joint", default="", help="joint name recorded in telemetry")

    ap.add_argument("--step", type=float, default=5.0,
                    help="step size in degrees (default 5). Must satisfy kp*step <= "
                         "--max-torque and step < --max-track-err")
    ap.add_argument("--sweep", help='comma-separated kp values, e.g. "0.61,0.85,1.22"')
    ap.add_argument("--kd-sweep", help="comma-separated kd values (paired with --sweep, or "
                                       "swept on its own against --kp)")
    ap.add_argument("--report", help="analyse an existing run folder and exit (no bus)")
    ap.add_argument("--json", action="store_true", help="--report: also dump raw metrics JSON")

    ap.add_argument("--kp", type=float, default=1.22, help="N.m/rad (default raw 10 = 1.22)")
    ap.add_argument("--kd", type=float, default=0.0098, help="N.m.s/rad (default raw 8)")
    ap.add_argument("--rate", type=float, default=50.0, help="command rate, Hz")
    ap.add_argument("--hold", type=float, default=1.0, help="s held before the step")
    ap.add_argument("--settle", type=float, default=3.0, help="s held after the step")
    ap.add_argument("--return-time", type=float, default=2.0, help="s to ramp back")
    ap.add_argument("--rest", type=float, default=3.0,
                    help="s between sweep runs (motor free, shaft settles)")

    ap.add_argument("--p-max", type=float, default=12.5)
    ap.add_argument("--v-max", type=float, default=200.0)
    ap.add_argument("--t-max", type=float, default=10.0)
    ap.add_argument("--max-torque", type=float, default=0.3)
    ap.add_argument("--max-track-err", type=float, default=12.0)
    ap.add_argument("--max-temp", type=int, default=60)
    ap.add_argument("--max-setpoint-vel", type=float, default=1.0)
    ap.add_argument("--max-shaft-vel", type=float, default=3.0)
    ap.add_argument("--soft-limits", metavar="LO,HI")
    ap.add_argument("--soft-limit-margin", type=float, default=2.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--no-log", action="store_true")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.report:
        return cmd_report(args)

    signal.signal(signal.SIGINT, g._on_signal)
    signal.signal(signal.SIGTERM, g._on_signal)

    kps = parse_gain_list(args.sweep) or [args.kp]
    kds = parse_gain_list(args.kd_sweep) or [args.kd]
    runs = [(kp, kd) for kp in kps for kd in kds]
    if len(runs) > 1:
        print(f"Sweep: {len(runs)} runs -- " +
              ", ".join(f"kp={kp} kd={kd}" for kp, kd in runs))

    rc = 0
    for i, (kp, kd) in enumerate(runs):
        if g._stop_requested:
            print("stop requested -- ending sweep")
            break
        if i:
            print(f"\n--- resting {args.rest:.1f}s (motor free) before run {i + 1}/{len(runs)} ---")
            time.sleep(args.rest)
        suffix = f"-run{i + 1}" if len(runs) > 1 else ""
        rc = bench_once(args, kp, kd, suffix) or rc
    return rc


if __name__ == "__main__":
    sys.exit(main())
