# Testing joint limits, velocity limits, and telemetry plots

> **Running the test?** Use the step-by-step [MIT_CLAMP_TEST.md](MIT_CLAMP_TEST.md): zero at a
> hand-set pose → move over a duration → hold → return, with over-extended and too-fast requests.
> This file is the background.

How to prove the arm's safety limits are actually enforced, and how to see it in a plot.
Covers all **seven** motors: the five AK arm joints, the GL40 wrist, and the GL40 gripper.

Two independent paths reach a motor, and both are tested here:

| Path | What it exercises | Tool |
|---|---|---|
| **ROS pipeline** | `ArmPose` → `joint_command` (clamp, velocity limit, smoothing, MIT gains, watchdog) → `MotorCmd` → `can_node` → CAN | `tools/gl40_ros_move.sh` |
| **Bench script** | one drive, raw SocketCAN, no ROS — for gain tuning and protocol work | `tools/gl40_move.sh` |

> ⚠️ **HARDWARE E-STOP.** Everything in "On real hardware" moves real motors. A physical
> cutoff on the motor supply must be within reach; the software stops here (exit-motor-mode,
> watchdog faults, aborts) are a complement, not a replacement. If the control PC freezes, only
> the hardware switch saves the arm.

---

## The seven motors

From `joint_command/config/hardware_mapping.yaml` (left arm):

| Motor | Joint | Model | Control | Driven by |
|---|---|---|---|---|
| 14 | shoulder.pitch | AK10-9 | POSITION_LOOP | `ArmPose` slot 0 |
| 12 | shoulder.roll | AK10-9 | POSITION_LOOP | slot 1 |
| 11 | shoulder.yaw | AK80-9 | POSITION_LOOP | slot 2 |
| 10 | elbow.pitch | AK80-9 | POSITION_LOOP | slot 3 |
| 13 | elbow.roll | AK80-9 | POSITION_LOOP | slot 4 |
| 22 | wrist.pitch | GL40 KV70 | **MIT_CONTROL** | slot 5 |
| 21 | gripper.open_close | GL40 KV70 | **MIT_CONTROL** | *not in `ArmPose`* — bench script or direct `MotorCmd` |

`ArmPose` carries six joints, so `joint_command` drives motors 14/12/11/10/13/22. The gripper
(21) has no `ArmPose` slot; drive it with `gl40_mit_move.py --id 21`, and telemetry records it
like any other motor.

---

## 0. Start the stack

```bash
cp watod-config.sh watod-config.local.sh     # ACTIVE_MODULES="interfacing"
./watod up -d
```

### Without hardware (recommended first)

`gl40_sim.py` emulates all seven drives on a virtual CAN bus — five AK in servo mode, two GL II
in MIT mode, each with inertia, damping and a gravity-like load. Everything below works against
it, and nothing can be damaged.

```bash
# vcan0 once per boot (needs NET_ADMIN -- the interfacing container is privileged)
docker exec -u root $(docker ps -qf name=-interfacing-) bash -c \
  'ip link add dev vcan0 type vcan 2>/dev/null; ip link set up vcan0'

# the seven emulated drives
docker exec -d -u root $(docker ps -qf name=-interfacing-) bash -c \
  'python3 /root/ament_ws/src/interfacing/can/scripts/gl40_sim.py --iface vcan0 > /tmp/sim.log 2>&1'

# can_node against vcan0 instead of the CANable
docker exec -d $(docker ps -qf name=-interfacing-) bash -lc \
  'source /opt/watonomous/setup.bash && ros2 run can can_node --ros-args \
   -r __node:=can_node_vcan -p can_interface:=vcan0 -p bustype:=socketcan > /tmp/can_vcan.log 2>&1'
```

Useful simulator flags: `--gl-start-deg` (where the wrist starts; the real bench wrist reads
~143.5°, which the placeholder limits exclude — see §4), `--ak-start-cmd-deg`, `--load-gain`
(how hard the gravity load pulls back), `--feedback-hz`.

### With hardware

`can.launch.py` starts automatically with the container and brings up `can0` from `/dev/canable`
at 1 Mbps. Confirm feedback is flowing before commanding anything:

```bash
./watod -t interfacing
source /opt/watonomous/setup.bash
ros2 topic hz /interfacing/motorFeedback     # ~50 Hz per AK motor
```

Then start the command node:

```bash
./watod -t joint_command
source /opt/watonomous/setup.bash
ros2 launch joint_command joint_command.launch.py
```

---

## 1. The limits, before you test them

Two files decide what "enforced" means:

**`joint_command/config/hardware_mapping.yaml`** — per joint `lower_limit` / `upper_limit`
(command-frame degrees), `direction`, `zero_offset`, and `limit_range`.

**`joint_command/config/safety_limits.yaml`** — what the pipeline does each control tick:

```
clamp -> low-pass -> velocity limit -> delta limit -> clamp (itself rate-limited) -> calibration
```

`velocity_max` is a true **degrees per second** bound: moderation runs on the 50 Hz control
tick, not per incoming message, so publishing `ArmPose` faster does not move the arm faster.
Shipped values are a deliberate crawl (10 °/s).

The node **refuses to start** if a MIT joint's gains break the stall rule
(`quantised mit_kp × mit_max_track_err ≤ mit_max_torque`), so a bad edit fails loudly at launch
rather than on the motor.

---

## 2. Test the limits without any motor (fast, every time)

```bash
./watod -t interfacing
sudo bash -c 'source /opt/ros/humble/setup.bash && source /opt/watonomous/setup.bash && \
  cd /root/ament_ws && colcon build --packages-select joint_command can && \
  colcon test --packages-select joint_command can && colcon test-result --verbose'
```

(`sudo` because `/root/ament_ws` is root-only in the container; the `watonomous` overlay is
where the prebuilt `common_msgs` lives, so sourcing it is what lets these two packages find it.)

These gtests run against the **shipped config files**, so they fail if someone disables the
position clamp, raises a velocity past the 2 rad/s testing ceiling, or breaks the gain rule:

| Test | Proves |
|---|---|
| `PositionClampIsEnabledInTheShippedConfig` | the clamp is actually on |
| `CommandsNeverLeaveTheConfiguredJointLimits` | 2000 ticks of a ±720° command never exit the limits |
| `VelocityMaxIsADegreesPerSecondBound` | one second of streaming moves exactly `velocity_max` degrees |
| `SpeedDoesNotDependOnHowOftenArmPoseArrives` | publisher rate does not change arm speed |
| `FirstCommandRampsFromTheMeasuredPoseNotFromZero` | no startup slam |
| `JointFoundOutsideItsOwnLimitsIsFlaggedAndExcluded` | a mis-calibrated joint is excluded, not clamped |
| `UnsafeMitGainsAreRefusedAtLoadTime` | kp 1.46 at 12° is rejected; 1.34 is allowed |
| `MitWatchdogCatchesEveryFaultCondition` | torque / tracking / timeout / drive-fault all trip |
| `test_mit_protocol` (can) | frame packing matches the manual, and the DBC field order matches the protocol |

Lint failures from `ament_copyright` / `cpplint` / `uncrustify` are pre-existing repo-wide and
unrelated; CI runs `autopep8` and `clang-format` instead.

---

## 3. Test clamping on a running system

Command far past every limit and watch where the arm actually stops.

```bash
# six command-frame angles: shoulder pitch,roll,yaw, elbow pitch,roll, wrist pitch
tools/gl40_ros_move.sh --pose "200,200,200,200,200,200" --duration 14 --label clamp-all-limits
```

The wrapper publishes the pose at 50 Hz, records telemetry for every motor, stops, and plots.
**Expected:** each joint ramps at `velocity_max` and stops on its own `upper_limit`
(shoulder.pitch 10.7°, shoulder.roll 3.7°, shoulder.yaw 38.2°, elbow.pitch 47.2°,
elbow.roll 36.0°, wrist 90°) — never past it, and never in a jump.

Read it off `angle.png`: each trace flattens exactly on its dashed red limit rule. `summary.md`
gives the same thing numerically, including a pass/fail line per check.

Other clamp cases worth running:

```bash
tools/gl40_ros_move.sh --pose "-200,-200,-200,-200,-200,-200" --label clamp-lower   # lower limits
tools/gl40_ros_move.sh --pose "0,0,0,0,0,200" --label clamp-wrist-only              # MIT joint alone
```

On the bench script, the equivalent is `--soft-limits`:

```bash
tools/gl40_move.sh --id 22 --deg 40 --soft-limits 120,150                  # refuses out of range
tools/gl40_move.sh --id 22 --deg 40 --soft-limits 120,150 --clamp-target   # clamps and stops at 150
```

---

## 4. Test the velocity limit

```bash
tools/gl40_ros_move.sh --pose "0,0,30,0,0,0" --duration 12 --rate 50  --label vel-50hz
tools/gl40_ros_move.sh --pose "0,0,30,0,0,0" --duration 12 --rate 200 --label vel-200hz
uv run --with matplotlib --with numpy tools/gl40_telemetry_plot.py \
  --sweep outputs/gl40_bench/*vel-50hz outputs/gl40_bench/*vel-200hz
```

**Expected:** both runs show the same slope in `velocity.png` — about 10 °/s, under the dashed
ceiling — because the limiter runs on the control tick. A 4× difference would mean the per-tick
fix has regressed.

---

## 5. What the plots show

Every run writes `outputs/gl40_bench/<UTC-timestamp>_<label>/`:

| File | Contents |
|---|---|
| `telemetry.csv` | one row per tick per motor: `t_s, source, motor_id, joint, phase, sp_deg, pos_deg, vel_dps, tau_nm, current_a, drive_c, motor_c, status` |
| `run.json` | gains as requested **and** as the drive quantises them, limits, rates, git sha, abort reason |
| `angle.png` | **commanded vs measured angle per motor**, with joint limits as dashed rules, phase bands, steady-state error, and the abort marked |
| `velocity.png` | joint velocity, differentiated from the measured angle, against the configured ceiling |
| `tracking.png` | `|commanded − measured|` against the fault threshold |
| `torque.png` | torque (MIT drives) and current (servo drives) against their ceilings |
| `summary.md` | every number plus pass/fail per check |

Velocity is **differentiated from position**, not taken from the drives: the reported velocity
depends on a parameter-page scale that is easy to get wrong, while the position scale is
verified with `--monitor`.

Logging is on by default — no flag needed. Plotting runs on the host because the robot-control
images deliberately carry no matplotlib/numpy; `uv` supplies them per invocation:

```bash
uv run --with matplotlib --with numpy tools/gl40_telemetry_plot.py outputs/gl40_bench/<run>
```

Record without moving anything (e.g. while someone hand-moves the arm):

```bash
docker exec $(docker ps -qf name=-joint_command-) bash -lc \
  'source /opt/watonomous/setup.bash && python3 /opt/humanoid_scripts/telemetry_record.py \
   --duration 30 --motors 10,11,12,13,14,21,22 --label hand-move'
```

---

## 6. The GL40s (motors 22 and 21)

These are PD drives: the motor applies `torque = kp·(target − actual) + kd·(0 − velocity)` and
has **no internal limit checking**, so `joint_command` supplies the protection —
`mit_max_torque`, `mit_max_track_err`, `mit_feedback_timeout`, each of which frees the drives
and latches until the node restarts.

Shipped gains (bench-measured 2026-09-19 on motor 22): **kp 1.22 → raw 10 → 1.2207 N·m/rad
applied**, **kd 0.0098 → raw 8**. Stalled-worst-case torque 0.256 N·m, under the 0.3 N·m ceiling.

Startup sequence, visible in the node log: `MIT_ENTER` → zero-gain frames while unseeded (a GL
II only answers when spoken to, and zero gains cannot move it) → gains applied once seeded from
real feedback → `MIT_EXIT` on Ctrl-C, on a stale command stream, and on any fault.

Tune gains with a step response (a slow ramp keeps the error near zero, so every gain looks the
same):

```bash
GL40_TOOL=gl40_bench.py tools/gl40_move.sh --id 22 --step 5              # one step
GL40_TOOL=gl40_bench.py tools/gl40_move.sh --id 22 --step 5 \
  --sweep "0.61,0.85,1.22,1.34" --kd-sweep "0.0098,0.0244" --max-track-err 12
```

Pick the highest kp the stall rule allows (raw 11 = 1.34 at 12°/0.3 N·m) that shows no
overshoot, then **write it into `safety_limits.yaml`** — that file, not the script, is what the
robot uses. A few degrees of steady-state sag under gravity is expected; pure PD under a torque
ceiling cannot remove it.

The gripper (21) is not in `ArmPose`, so exercise it directly:

```bash
tools/gl40_move.sh --id 21 --monitor                       # check its scale by hand
tools/gl40_move.sh --id 21 --deg 20 --kp 1.22 --max-track-err 12 --joint gripper.open_close
```

### "EXCLUDING joint …" in the log

If a joint's measured position is outside its own configured limits, its calibration and the
hardware disagree. That joint is **excluded** (MIT ones held limp) and named in an error; the
rest of the arm still works. Clamping it instead would walk it to the limit the moment it was
commanded — motion nobody asked for.

This is expected today for the wrist on real hardware: `hardware_mapping.yaml` still carries
placeholder ±90° limits for motors 22/21, and the real wrist sits near 143°. Fix it by running
`calibrate_arm.py` for those joints; until then the wrist will be excluded from ROS-driven
motion (the bench script, which does not use that file, still drives it).

---

## 7. On real hardware, in order

1. `tools/gl40_move.sh --id 22 --selftest` and the gtests in §2. No bus is touched.
2. `tools/gl40_move.sh --id 22 --monitor`: zero torque. Turn the shaft by hand; a quarter turn
   must read ≈ 1.571 rad. This is what verifies `--p-max` and the velocity scale.
3. Calibrate (`calibrate_arm.py`), then **rebuild joint_command**
   (`./watod build joint_command && ./watod up -d`). The node loads its installed copy of
   `hardware_mapping.yaml` / `safety_limits.yaml`, and `arm_roundtrip.py` refuses to run while
   that copy differs from the repo's.
4. Angle round trips through the teleop path, one joint at a time, lightest load first. Each run
   goes out, dwells, comes back to its start and settles:
   ```bash
   tools/arm_roundtrip.sh --joints elbow.roll     --offset "0,0,0,0,5,0"
   tools/arm_roundtrip.sh --joints elbow.pitch    --offset "0,0,0,5,0,0"
   tools/arm_roundtrip.sh --joints shoulder.yaw   --offset "0,0,5,0,0,0"
   tools/arm_roundtrip.sh --joints shoulder.roll  --offset "0,1.5,0,0,0,0"   # range -7.7..3.7
   tools/arm_roundtrip.sh --joints shoulder.pitch --offset "3,0,0,0,0,0"     # most gravity load
   tools/arm_roundtrip.sh --joints wrist.pitch    --offset "0,0,0,0,0,10"
   ```
   Grow the offsets toward the benchmark angles, then move all joints together. `summary.md`
   gives `benchmark_deg`, `steady_state_err_deg` (sag at the target) and `return_err_deg`.
5. §3 clamp tests, starting with one joint and a small target.
6. §4 velocity test.
7. §6 gain sweep, then write the chosen gains into `safety_limits.yaml` and rebuild.
8. Keep every run folder. `angle.png` and `summary.md` are the evidence that the limits held.

Before each: hardware E-stop within reach, arm clear, nothing else publishing to
`/arm/joint_targets`, and — for the AK80-9 joints (elbow pitch/roll, shoulder yaw) — recalibrate
after every power cycle, since they lose their zero (`ARM_BRINGUP.md`).
