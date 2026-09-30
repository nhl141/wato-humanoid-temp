# Tuning gravity feed-forward and kp: bench guide

No URDF knowledge needed. You need the arm, the e-stop, and a terminal in the repo root.

## The idea

- In MIT mode each joint behaves like a spring: `torque = kp × (target − actual)`. The spring
  has to stretch before it pushes back, so the arm's own weight makes each joint **sag** below
  its target.
- **Feed-forward (FF)** adds the torque needed to hold the arm's weight up front, so the spring
  has nothing left to hold. That torque comes from a CAD model of the arm.
- The model measures every angle from its own **zero pose: arm hanging straight down**.
  Steps 1–2 make your angles agree with it. Step 3 measures how heavy the real arm is compared
  with CAD (a factor **k**). Step 4 turns FF on. Step 5 makes the joints stiffer.

| Joint | Worst gravity load | Needs FF? |
|---|---|---|
| shoulder pitch | ~4.8 N·m × k | **yes** |
| shoulder roll | ~4.7 N·m × k | **yes** |
| elbow pitch | ~1.2 N·m × k | **yes** |
| shoulder yaw | ~1.2 N·m, only when reaching | later / optional |
| elbow roll, wrist pitch | ~0.2 N·m | no, only tune kp (step 5) |

Tune in this order: **elbow pitch → shoulder roll → shoulder pitch**.

## Every session

- Hardware e-stop within reach. Hold or support the arm whenever no roundtrip is running.
  Idle MIT joints are soft: the shoulders and elbows sink slowly, and the wrist is limp.
- `can_node` (interfacing container) and `joint_command` must be running.
- **Once, after pulling these changes:** `./watod build joint_command && ./watod up -d joint_command`
- **After every edit to `config/safety_limits.yaml`:** `./watod restart joint_command`.
  If the node then refuses to start, `./watod logs --tail 30 joint_command` says why. It is
  almost always the gain rule; see [Troubleshooting](#troubleshooting).

## Reading the arm's angles

These are "cmd angles", the same numbers that `arm_roundtrip.sh --pose` uses. This only listens;
it sends no commands:

```bash
C=$(docker ps --filter name=-joint_command- --format '{{.Names}}' | head -1)
docker exec $C bash -lc 'source /opt/watonomous/setup.bash; python3 /opt/humanoid_scripts/telemetry_record.py --duration 2 --label where' >/dev/null
python3 -c "import csv,glob,os; f=max(glob.glob('outputs/gl40_bench/*_where/telemetry.csv'),key=os.path.getmtime); d={r['joint']:r['pos_deg'] for r in csv.DictReader(open(f))}; print('\n'.join(f'{k:16}{float(v):+7.1f}' for k,v in sorted(d.items()) if v) or 'no feedback: motors powered? can_node running?')"
```

## Step 1: calibrate in the zero pose (every power-on, as you do now)

Run `calibrate_arm.py` the usual way (can/README.md). At each "move joint to the pose you want
as 0°" prompt, put that joint in its **zero pose**:

| Joint | Zero pose |
|---|---|
| shoulder pitch | upper arm hanging straight down, not forward or back |
| shoulder roll | upper arm hanging straight down, not out to the side |
| shoulder yaw | elbow hinge pin points left–right, so bending the elbow swings the forearm forward/back |
| elbow pitch | elbow straight |
| elbow roll | wrist hinge pin points left–right, so the hand swings forward/back |
| wrist pitch | hand straight, in line with the forearm |

Now every joint's `urdf_offset_deg: 0.0` in `safety_limits.yaml` is correct; leave it.
**Check:** hold the arm in the zero pose and read the angles. Every joint should read within ~±3°.

This changes what "0" means for your roundtrip poses: 0 is now "hanging straight". If you'd
rather keep your old calibration, hold the zero pose, read the angles, and set each joint's
`urdf_offset_deg` to −(reading) if its `urdf_direction` is 1, or +(reading) if it is −1.

## Step 2: direction check (once; redo if you change a `direction` in hardware_mapping.yaml)

Start from the zero pose. Move **one** joint about 20° by hand as described, then read its angle
again:

| Joint | From the zero pose, move it… | `urdf_direction: 1` if the angle goes… |
|---|---|---|
| shoulder pitch | arm forward | up |
| shoulder roll | standing in front of the robot facing it: swing the arm **counter-clockwise** (it swings up toward your right; the direction with ~90° of room, not the ~20° side blocked by the body) | up |
| shoulder yaw | first bend the elbow 90° forward, then swing the forearm inward across the chest | up |
| elbow pitch | bend the elbow the normal way (forearm forward) | **down** |
| elbow roll | twist the forearm counter-clockwise, seen from above | up |
| wrist pitch | bend the hand forward | **down** |

If the angle went the other way, set `urdf_direction: -1` for that joint. Delete the
`# UNVERIFIED` comments once each joint is checked, then restart the node.

## Step 3: measure the load (FF still off)

`gravity_ff_scale` must still be 0 (the default). The joint under test must be on
`control_type: 0`.

**Other joints:** easiest is to power all six. If a joint is unpowered, strap it in its zero
pose and set `gravity_assume_deg: 0.0` in its block. Remove that again when you're done.

Run each pose. `--pose` takes six numbers in the order
`shoulder pitch, shoulder roll, shoulder yaw, elbow pitch, elbow roll, wrist pitch`; only the
`--joints` joint moves.

```bash
# elbow pitch (forearm forward = negative)
tools/arm_roundtrip.sh --joints elbow.pitch --pose "0,0,0,-30,0,0" --dwell 5 --max-track-err 11 --max-delta 95 --label gff-ep-30
#   ...then -60, -90
# shoulder roll (out to the side = positive)
tools/arm_roundtrip.sh --joints shoulder.roll --pose "0,10,0,0,0,0" --dwell 5 --max-track-err 11 --label gff-sr10
#   ...then 20, 30
# shoulder pitch (forward = positive; backward poses help too)
tools/arm_roundtrip.sh --joints shoulder.pitch --pose "10,0,0,0,0,0" --dwell 5 --max-track-err 11 --label gff-sp10
#   ...then 15, -10, -15
```

A run that aborts on tracking error means that pose sags too much without FF. Skip it; step 4
reaches it.

Then fit:

```bash
uv run tools/gravity_fit.py outputs/gl40_bench/*_gff-ep*
```

How to read the output:

- **ratio** per run (measured ÷ model): these should roughly agree (±20%).
- **k**: the fitted factor. That's your `gravity_ff_scale`.
- **suggest … gravity_ff_max_torque**: the FF cap to use.
- **gain rule … OK / VIOLATES**: whether that cap fits under the joint's torque ceiling.

Red flags:

- **k negative:** go back to step 2 (direction).
- **Ratios differ a lot between poses:** go back to step 1 (zero pose).
- **k > 2:** check steps 1–2 first, then ask. It's beyond what the node allows.

## Step 4: turn FF on

In that joint's block in `safety_limits.yaml`:

```yaml
gravity_ff_scale: 0.9          # e.g. k = 1.8 -> start at HALF of k
gravity_ff_max_torque: 5.5     # e.g. -- use the fit's suggestion
```

**If the fit said VIOLATES, stop here and decide with the team.** Either narrow that joint's
range, or raise its `mit_max_torque` (AK10-9 is rated 18 N·m; the testing ceiling is 10).
Don't just push numbers until the node starts.

Restart the node. Re-run the step 3 poses, plus further ones:

- shoulder roll: 45, 60
- shoulder pitch: 30, 45; 60 only if the gain rule allows (these need `--max-delta 65`)

Fit again over all of that joint's runs (k should stay about the same), set
`gravity_ff_scale: <k>`, restart, and repeat the poses.

**Pass:** no aborts, and in each run folder's `summary.md`, `benchmark_deg` is within ~2° of
`requested_deg`.

## Step 5: stiffen kp (all joints, including elbow roll and wrist)

With FF holding the weight, kp only fights what's left, so raising it tightens the joint. Repeat:

1. `mit_kp` × 1.25 and `mit_kd` × 1.12 (keeps damping about the same).
2. Restart, then run two poses from step 3/4.
3. Stop and step back one notch if you see:
   - buzzing or vibration while holding
   - overshoot at the end of the move (`tracking.png`)
   - drive temperature climbing past ~50 °C
   - the node refusing to start

When the node refuses (gain rule), you can free up room by lowering `mit_max_track_err` for that
joint (12 → 8 → 6), but only once its held error is small. Rough targets:

| Joint | mit_kp | mit_max_track_err |
|---|---|---|
| shoulders | 25–30 | 6 |
| elbows, shoulder yaw | 12–15 | 6 |
| wrist | leave as is | 12 |

## Step 6: final checks and record

- A few combined poses, e.g.
  `tools/arm_roundtrip.sh --joints shoulder.pitch,elbow.pitch --pose "30,0,0,-60,0,0" --dwell 5 --max-delta 65 --label gff-combo`
- Stop one mid-hold with **Ctrl-C twice**: the arm must sink slowly, not drop.
- In `safety_limits.yaml`, next to each tuned joint, write k, the final gains, the date and the
  run folders. Remove any `gravity_assume_deg` you added.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Node won't start after an edit | Gain rule: `kp × track_err + ff_max > mit_max_torque` (see logs) | Lower `mit_kp`, `mit_max_track_err` or `gravity_ff_max_torque` |
| Arm sags *more* or pushes the wrong way with FF on | Wrong `urdf_direction` | Set `gravity_ff_scale: 0`, redo step 2 |
| Fit: "no feedback for X and no gravity_assume_deg" | X was unpowered | Power X, or strap it and set its `gravity_assume_deg` |
| Fit: "still moving at dwell end" | Dwell too short | Use `--dwell 8` |
| Fit: "no dwell phase" | Run aborted before reaching the target | Normal without FF; use a smaller pose |
| Node log: "Gravity model ASSUMES …" | A `gravity_assume_deg` is set | Fine while tuning; remove it afterwards |
