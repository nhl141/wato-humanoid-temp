#!/usr/bin/env python3
# /// script
# dependencies = ["pyyaml"]
# ///
"""Fit gravity feed-forward per joint from arm_roundtrip dwell data. Analysis only.

For each run it takes the last second of the dwell, per joint, and compares the torque the joint
really spent holding the arm against the URDF gravity model (the same model as
joint_command/src/gravity_model.cpp, parsed here from the URDF directly):

  MIT joint    tau_meas = kp * direction * (sp - pos) + ff_nm      (drive's own torque units)
  servo joint  tau_meas = tau_nm (current x kt)                    (lower trust: kt unverified)
  tau_pred     = direction * urdf_direction * tau_model(q_urdf),  q_urdf = urdf_direction * q + urdf_offset_deg

all in the motor frame. Per joint it fits k = sum(meas*pred) / sum(pred^2) -- the factor between
the real load and the CAD model, i.e. the gravity_ff_scale to use -- and checks the startup gain
rule for the suggested gravity_ff_max_torque. Approach each pose from below AND above: Coulomb
friction then cancels in the fit, and the spread between the pair estimates it.

  uv run tools/gravity_fit.py outputs/gl40_bench/<run> [<run> ...]
  uv run tools/gravity_fit.py outputs/gl40_bench/*rt-shouldp* --assume elbow.pitch=0

Joints with no feedback in a run use their gravity_assume_deg (or --assume); without one the run
is skipped, exactly as joint_command zeroes feed-forward then.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "interfacing" / "can" / "scripts"))
import joint_config as jc  # noqa: E402
import telemetry  # noqa: E402

URDF = REPO / "assets" / "pioneer_bimanual_arm" / "urdf" / "pioneer_bimanual_arm.urdf"
CHAIN = ["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"]  # ArmPose order
FINGERS = ["joint7l", "joint8l"]
GRAVITY = 9.81
MIN_PRED_NM = 0.3       # samples below this carry more friction than load
DWELL_WINDOW_S = 1.0
MAX_DWELL_DRIFT_DPS = 0.5
KP_STEP = 500.0 / 4096.0  # 12-bit MIT gain code, as JointCommandCore::quantiseKp

Vec = Tuple[float, float, float]


def _vec(text: str) -> Vec:
    x, y, z = (float(v) for v in text.split())
    return (x, y, z)


def _add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _matvec(m, v: Vec) -> Vec:
    return (_dot(m[0], v), _dot(m[1], v), _dot(m[2], v))


def _matmul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def _axis_angle(k: Vec, q: float):
    s, c = math.sin(q), 1.0 - math.cos(q)
    K = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
    KK = _matmul(K, K)
    return [[(1.0 if r == col else 0.0) + s * K[r][col] + c * KK[r][col] for col in range(3)]
            for r in range(3)]


class ArmModel:
    """Left-arm static gravity load from the URDF's CAD masses (fingers lumped at zero)."""

    def __init__(self, urdf: Path = URDF):
        root = ET.parse(urdf).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        joints = {j.get("name"): j for j in root.findall("joint")}

        def inertial(link_name: str) -> Tuple[float, Vec]:
            node = links[link_name].find("inertial")
            return float(node.find("mass").get("value")), _vec(node.find("origin").get("xyz"))

        self.links = []  # (origin, axis, mass, com)
        self.limits = []
        for name in CHAIN:
            j = joints[name]
            origin = j.find("origin")
            if any(float(v) for v in origin.get("rpy", "0 0 0").split()):
                raise SystemExit(f"{name}: non-zero rpy is not supported")
            mass, com = inertial(j.find("child").get("link"))
            if name == CHAIN[-1]:
                parts = [(mass, com)]
                for finger in FINGERS:
                    fj = joints[finger]
                    fm, fc = inertial(fj.find("child").get("link"))
                    parts.append((fm, _add(_vec(fj.find("origin").get("xyz")), fc)))
                mass = sum(m for m, _ in parts)
                com = tuple(sum(m * c[i] for m, c in parts) / mass for i in range(3))
            self.links.append((_vec(origin.get("xyz")), _vec(j.find("axis").get("xyz")), mass, com))
            lim = j.find("limit")
            self.limits.append((float(lim.get("lower")), float(lim.get("upper"))))

    def hold_torque(self, q_urdf_rad: List[float]) -> List[float]:
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        p: Vec = (0.0, 0.0, 0.0)
        pos, axes, coms = [], [], []
        for (origin, axis, _, com), q in zip(self.links, q_urdf_rad):
            p = _add(p, _matvec(R, origin))
            pos.append(p)
            axes.append(_matvec(R, axis))
            R = _matmul(R, _axis_angle(axis, q))
            coms.append(_add(p, _matvec(R, com)))
        tau = []
        for i in range(len(self.links)):
            moment: Vec = (0.0, 0.0, 0.0)
            for j in range(i, len(self.links)):
                weight = (0.0, 0.0, -self.links[j][2] * GRAVITY)
                moment = _add(moment, _cross(_sub(coms[j], pos[i]), weight))
            tau.append(-_dot(axes[i], moment))
        return tau

    def worst_case(self, samples: int = 4000) -> List[float]:
        """Largest |hold torque| per joint over a random sweep of the URDF limits."""
        rng = random.Random(0)
        best = [0.0] * len(self.links)
        for _ in range(samples):
            tau = self.hold_torque([rng.uniform(lo, hi) for lo, hi in self.limits])
            best = [max(b, abs(t)) for b, t in zip(best, tau)]
        return best


def _mean(values: List[float]) -> Optional[float]:
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def dwell_samples(run_dir: Path, model: ArmModel, joint_map: Dict[int, dict],
                  safety: Dict[str, dict], assume: Dict[str, float]):
    """-> (samples, skip_reason). One sample per joint that reported during the dwell."""
    _, rows = telemetry.load_run(run_dir)
    names = jc.ARM_POSE_NAMES
    by_joint: Dict[str, list] = {}
    for r in rows:
        if r.get("joint") in names:
            by_joint.setdefault(r["joint"], []).append(r)

    held = {}
    for name, jrows in by_joint.items():
        dwell = [r for r in jrows if r["phase"] == "dwell" and r["pos_deg"] is not None]
        if not dwell:
            continue
        t_end = dwell[-1]["t_s"]
        window = [r for r in dwell if r["t_s"] >= t_end - DWELL_WINDOW_S]
        dt = window[-1]["t_s"] - window[0]["t_s"]
        drift = abs(window[-1]["pos_deg"] - window[0]["pos_deg"]) / dt if dt > 0 else 0.0
        if drift > MAX_DWELL_DRIFT_DPS:
            return [], f"{name} still moving at dwell end ({drift:.2f} deg/s)"
        ramp = [r for r in jrows if r["phase"] == "ramp" and r["sp_deg"] is not None]
        sp = _mean([r["sp_deg"] for r in window])
        approach = ""
        if ramp and sp is not None:
            approach = "from below" if sp > ramp[0]["sp_deg"] else "from above"
        held[name] = {
            "pos": _mean([r["pos_deg"] for r in window]),
            "sp": sp,
            "ff": _mean([r.get("ff_nm") for r in window]),
            "kp": _mean([r.get("kp") for r in window]),
            "tau": _mean([r["tau_nm"] for r in window]),
            "approach": approach,
        }
    if not held:
        return [], "no dwell phase"

    q_cmd = []
    for name in names:
        if name in held:
            q_cmd.append(held[name]["pos"])
        elif name in assume:
            q_cmd.append(assume[name])
        elif safety.get(name, {}).get("gravity_assume_deg") is not None:
            q_cmd.append(float(safety[name]["gravity_assume_deg"]))
        else:
            return [], f"no feedback for {name} and no gravity_assume_deg (or --assume)"

    urdf_dir = [int(safety.get(n, {}).get("urdf_direction") or 1) for n in names]
    offset = [float(safety.get(n, {}).get("urdf_offset_deg") or 0.0) for n in names]
    tau_model = model.hold_torque([math.radians(d * q + o)
                                   for d, q, o in zip(urdf_dir, q_cmd, offset)])
    direction = {info["name"]: float(info.get("direction") or 1.0) for info in joint_map.values()}

    samples = []
    for i, name in enumerate(names):
        h = held.get(name)
        if h is None:
            continue
        d = direction.get(name, 1.0)
        pred = d * urdf_dir[i] * tau_model[i]
        if h["kp"] and h["sp"] is not None:
            meas = h["kp"] * d * math.radians(h["sp"] - h["pos"]) + (h["ff"] or 0.0)
            source = "mit"
        elif h["tau"] is not None:
            meas, source = h["tau"], "current*kt"
        else:
            continue
        samples.append({"run": run_dir.name, "joint": name, "pose": q_cmd, "q": h["pos"],
                        "approach": h["approach"], "pred": pred, "meas": meas,
                        "source": source, "ff": h["ff"]})
    return samples, None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("runs", nargs="+", type=Path, help="run folders (outputs/gl40_bench/...)")
    ap.add_argument("--mapping", help="hardware_mapping.yaml (default: the repo's)")
    ap.add_argument("--safety-limits", help="safety_limits.yaml (default: the repo's)")
    ap.add_argument("--arm-side", default="left")
    ap.add_argument("--assume", action="append", default=[], metavar="JOINT=DEG",
                    help="cmd-frame angle for a joint with no feedback; overrides the yaml's "
                         "gravity_assume_deg")
    args = ap.parse_args(argv)

    assume = {}
    for item in args.assume:
        name, _, deg = item.partition("=")
        if name not in jc.ARM_POSE_NAMES or not deg:
            sys.exit(f"--assume wants JOINT=DEG with JOINT in {jc.ARM_POSE_NAMES}")
        assume[name] = float(deg)

    joint_map = jc.load_joint_map(jc.find_mapping(args.mapping), args.arm_side)
    _, safety = jc.load_joint_safety(jc.find_safety_limits(args.safety_limits))
    model = ArmModel()

    samples = []
    for run in args.runs:
        got, why = dwell_samples(run, model, joint_map, safety, assume)
        if why:
            print(f"skip {run.name}: {why}")
        samples += got
    if not samples:
        print("no usable samples")
        return 1

    print(f"\n{'run':34} {'joint':15} {'q':>7} {'approach':11} {'source':10} "
          f"{'pred':>7} {'meas':>7} {'ratio':>6}")
    for s in samples:
        ratio = f"{s['meas'] / s['pred']:6.2f}" if abs(s["pred"]) >= MIN_PRED_NM else "     -"
        print(f"{s['run'][:34]:34} {s['joint']:15} {s['q']:7.1f} {s['approach']:11} "
              f"{s['source']:10} {s['pred']:7.2f} {s['meas']:7.2f} {ratio}")

    worst = dict(zip(jc.ARM_POSE_NAMES, model.worst_case()))
    print()
    for name in jc.ARM_POSE_NAMES:
        js = [s for s in samples if s["joint"] == name and abs(s["pred"]) >= MIN_PRED_NM]
        if not js:
            continue
        k = sum(s["meas"] * s["pred"] for s in js) / sum(s["pred"] ** 2 for s in js)
        sources = sorted({s["source"] for s in js})
        print(f"{name}: k = {k:.2f} from {len(js)} sample(s) [{', '.join(sources)}]")

        pairs: Dict[int, Dict[str, float]] = {}
        for s in js:
            if s["approach"]:
                pairs.setdefault(round(s["q"]), {})[s["approach"]] = s["meas"]
        spreads = [abs(p["from below"] - p["from above"]) / 2 for p in pairs.values()
                   if len(p) == 2]
        if spreads:
            print(f"  friction (half the below/above spread): ~{_mean(spreads):.2f} N.m")
        elif len(js) > 1:
            print("  no below/above pairs: friction is folded into k")

        ff_max = 1.15 * abs(k) * worst[name]
        block = safety.get(name, {})
        kp, err, ceiling = (block.get("mit_kp"), block.get("mit_max_track_err"),
                            block.get("mit_max_torque"))
        print(f"  suggest gravity_ff_scale: {k:.2f}   gravity_ff_max_torque: {ff_max:.2f} N.m "
              f"(model worst case {worst[name]:.2f} N.m x k x 1.15)")
        if k < 0:
            print("  k < 0: the model's sign disagrees with the arm -- fix urdf_direction / "
                  "urdf_offset_deg before enabling anything")
        elif k > 2:
            print("  k > 2: above joint_command's gravity_ff_scale limit -- check the URDF "
                  "mapping and kt before trusting this")
        if kp is not None and err is not None and ceiling is not None:
            kp_q = round(float(kp) / KP_STEP) * KP_STEP
            pd = kp_q * math.radians(float(err))
            verdict = "OK" if pd + ff_max <= float(ceiling) else "VIOLATES"
            print(f"  gain rule: {kp_q:.2f} x {float(err):g} deg = {pd:.2f} + {ff_max:.2f} "
                  f"<= {float(ceiling):g} N.m  {verdict}  (room for ff_max: "
                  f"{float(ceiling) - pd:.2f} N.m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
