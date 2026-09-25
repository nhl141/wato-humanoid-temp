# CAN interfacing (`can` package)

ROS 2 bridge: `/interfacing/motorCMD` ↔ CAN ↔ `/interfacing/motorFeedback`.
[Electrical docs](https://watonomous.github.io/humanoid-docs/electrical/index.html) · [Interfacing docs](https://watonomous.github.io/humanoid-docs/interfacing/index.html)

## Arm bring-up

1. **Hardware**: battery + E-stop closed (motor power ≠ CAN power). CANable USB → host; CAN_H/CAN_L → arm (120 Ω term).
2. **Host setup (once)**: `./src/interfacing/can/scripts/can_udev.sh install` → `/dev/canable`.
   `watod-config.local.sh`: `ACTIVE_MODULES="interfacing"`, `MODE_OF_OPERATION="develop"`.
3. **Bring up**:
   ```bash
   ./watod build && ./watod up -d
   ./watod -t interfacing
   source /opt/watonomous/setup.bash
   ```
   `can.launch.py` starts `can_node` + SLCAN (`/dev/canable` → `can0` @ 1 Mbps).

**gs_usb / candleLight adapter** (`lsusb` shows `1d50:606f` rather than `16d0:117e`): the kernel
exposes it as a native `can0`, so skip `can_udev.sh`/slcand. Bring the link up, then run the
node with `bustype:=socketcan`:
```bash
./src/interfacing/can/scripts/setup_socketcan.sh can0 1000000   # [--listen-only] to sniff only
ros2 run can can_node --ros-args --params-file $(ros2 pkg prefix can)/share/can/config/params.yaml \
  -p bustype:=socketcan
```

### Verify
```bash
ros2 node list                  # /can_node
candump can0                    # e.g. 0x290A–0x290E
ros2 topic echo /interfacing/motorFeedback common_msgs/msg/MotorFeedback --once
```

### Calibrate (`calibrate_arm.py`)
Per joint: confirm motor id → home zero → one end Enter → other end Enter → writes `zero_offset`/limits/`can_id`.
```bash
source /opt/watonomous/setup.bash
python3 /root/ament_ws/src/interfacing/can/scripts/calibrate_arm.py \
  --arm-side left --write-mapping --mapping /calibration/hardware_mapping.yaml
```
Prompt: **Enter**=yes · id=correct id · **s**=skip · **q**=quit.

### GL40 II in MIT mode (`gl40_mit_move.py`, `gl40_bench.py`)

The wrist (id 22) and gripper (id 21) GL40s use CubeMars' **GL II gimbal drive**, which in MIT
mode speaks *standard* 11-bit frames (`ID = node id`, feedback on the master id, default
`0x000`) with the byte order `pos(16) vel(12) kp(12) kd(12) t_ff(12)`
([manual §5](https://www.cubemars.com/images/file/20241231/1735633965678815.pdf)).

**`can_node` now drives them.** `humanoid.dbc` lays `MITControlCmd` out in the protocol's order
as a standard-id message, `config/mit_profiles.yaml` has entries for ids 21/22 (`family: gl2`),
`MotorCmd` carries `MIT_ENTER/EXIT/SET_ZERO/CLEAR_ERRORS` (the `FF..FC/FD/FE/FB` frames), and
MIT feedback on the master id is decoded into `MotorFeedback` (with `torque`). Gains sent as
`MotorCmd.kp/kd` are snapped to the drive's nearest 12-bit code — truncating, as the manual's
reference code does, would silently apply up to a full count less (kp 0.61 → 0.488).

The raw-SocketCAN bench tools remain, for what the ROS path cannot do: step-response gain
tuning, and the gripper (21), which has no `ArmPose` slot. Angle benchmarks belong on
`arm_roundtrip.py` (below), which goes through the same `joint_command` as teleop.

```bash
S=/root/ament_ws/src/interfacing/can/scripts   # /root/ament_ws is root-only -> sudo
sudo python3 $S/gl40_mit_move.py --selftest                 # packing vs the manual (no bus)
sudo python3 $S/gl40_mit_move.py --id 22 --monitor          # zero torque; check the rad scale
sudo python3 $S/gl40_mit_move.py --id 22 --deg 40 --dry-run # print frames only
sudo python3 $S/gl40_mit_move.py --id 22 --deg 40 --hold    # out, hold until Ctrl-C, back
sudo python3 $S/gl40_bench.py    --id 22 --step 5 --sweep "0.61,1.22,1.34"   # gain sweep
```

**Same limits as teleop.** Both tools load the files `joint_command` enforces:
- `kp`/`kd`, `--max-torque` and `--max-track-err` default to the joint's `mit_*` block in
  `safety_limits.yaml`.
- `--max-setpoint-vel` defaults to its `velocity_max` (10 °/s).
- `--soft-limits` defaults to its `hardware_mapping.yaml` range, converted to the drive frame.

A flag may tighten any of these, never loosen it. A joint with no validated gains (the gripper)
needs explicit `--kp`/`--kd`. AK ids are refused, because this is the GL II protocol only. A
joint already outside its limits is refused too, as `joint_command` would exclude it.
`--monitor` skips the limits, since it commands nothing. The one exception is the step in
`gl40_bench.py`: it is the measurement, so it is bounded by `kp·step ≤ max_torque` and
`step < max_track_err` instead of by `velocity_max`.

It reads the position first, holds it, ramps the setpoint, settles, then **ramps back to where it
started and only then frees the motor**, so a loaded joint is never dropped from the target.
The first Ctrl-C returns to the start as well; a second one frees immediately. `--no-return`
restores the old free-at-target behaviour. Safety aborts still free at once, because a drive that
just faulted can't be trusted to servo back. They fire on torque, tracking error, shaft velocity
> 3 rad/s, leaving `--soft-limits`, > 60 °C, a drive error, or 200 ms without feedback.
`--clamp-target` clamps an out-of-range target instead of refusing it; that is how limit
enforcement is demonstrated. Gains must satisfy `kp × max-track-err ≤ max-torque`: a stalled
motor is the worst case, since the tracking abort caps the PD torque.

**Gains, and where they live now.** The codebase is the source of truth:
`joint_command/config/safety_limits.yaml` holds the per-joint `mit_kp`/`mit_kd` that
`joint_command` sends through `MotorCmd`, and the node refuses to start if they violate the
rule above. Bench result 2026-09-19 on id 22: kp 0.366 lagged > 15° and aborted; kp 0.49 held
with a 6° sag; **kp 1.22 (raw 10) / kd 0.0098 (raw 8)** with a 12° abort limit moved +34° and
held steady at 0.125 N·m — those are the shipped values. Expect a few degrees of steady-state
sag under a gravity load: pure PD under a 0.3 N·m ceiling cannot do better, and closing it
needs torque feed-forward (not implemented).

`--p-max/--v-max/--t-max` must match the drive's parameter page (defaults ±12.5 rad / ±200 /
±10 N·m) — `--monitor` is how you check `--p-max`.

### AK joints in MIT mode (not yet brought up)

Every AK joint ships on `POSITION_LOOP`. The code for running one on `MIT_CONTROL` is in place:
- `decodeAkFeedback()`, which `can_node` dispatches by exact id; AK ids that collide with a GL II
  id nibble are refused at load.
- AK-aware fault status (`mit_family: ak`).
- A `damp` fault action, so a loaded joint sinks instead of dropping.
- Commented starting gains in `safety_limits.yaml`.

`gl40_sim.py --ak-mode mit` exercises all of it with a gravity load and the drive's CAN timeout.
**Before uncommenting a joint's MIT block, on the bench:**

1. Put the drive in MIT mode in the CubeMars upper computer, and set its CAN timeout
   (R-Link) to 100–200 ms. Find out whether this firmware can switch modes over CAN at all.
2. `candump can0` while sending `MIT_ENTER` and a zero-stiffness command. Confirm the reply comes
   on the master id with the full id in byte 0, and check the temperature and error bytes. Then
   replace the synthetic frame in `test/test_mit_protocol.cpp` (`DecodesAkLayoutFromTheManual`)
   with the capture. That layout is from the manual only.
3. Read the joint at one pose in servo mode and in MIT mode. If the frames differ,
   `hardware_mapping.yaml`'s `zero_offset` is wrong for MIT. AK80-9s (ids 10, 11, 13) lose zero
   on every power cycle in either mode.
4. Support the arm. The drive's CAN timeout cuts output and drops it whatever `joint_command`
   does. Test `damp` by killing the `ArmPose` stream, then by stopping `joint_command`.
5. One joint at a time: elbow roll (13), elbow pitch (10), shoulder yaw (11), shoulder roll (12),
   shoulder pitch (14). Hold 30 s, then ±5° moves, watching torque / tracking error / temperature.

Pure PD sags by about τ_gravity / kp. On the shoulders that can exceed `mit_max_track_err` within
the torque ceiling (AK10-9 10 N·m, AK80-9 5 N·m), and the watchdog faults. Closing that gap needs
gravity feed-forward in `MotorCmd.torque` (not implemented).

### Angle benchmarks through the teleop path (`arm_roundtrip.py`)

Step-by-step clamp test for every motor: [MIT_CLAMP_TEST.md](../MIT_CLAMP_TEST.md).

`tools/arm_roundtrip.sh` runs `scripts/arm_roundtrip.py` in the `joint_command` container. It
publishes `ArmPose` to `/arm/joint_targets` exactly as Quest / `task_space_ik` do, so every
limit teleop runs under applies: the clamp, `velocity_max`, the low-pass, and the MIT gains and
watchdog. It never talks to a drive directly.

```bash
tools/arm_roundtrip.sh --joints elbow.roll --offset "0,0,0,0,5,0"    # one joint, +5 deg
tools/arm_roundtrip.sh --offset "3,1.5,5,5,5,10" --dwell 5            # all six at once
```

**Sequence:**
1. Read all six joints' origin from feedback.
2. **hold** at the origin, and check that `joint_command` is commanding the measured pose.
3. **ramp** out on a cosine profile. Velocity is zero at both ends, and the peak is at most
   `--vel` (5 °/s) and at most each joint's `velocity_max`.
4. **dwell** at the target, then check each moving joint covered at least half its move. This
   flags a drive that never enabled, which a small move would not trip the tracking limit for.
5. **return** on the same profile.
6. **rest** at the origin.

**It refuses before moving if:**
- another node publishes `/arm/joint_targets` (e.g. teleop);
- a joint in `--joints` is silent. Silent joints you are not moving are allowed, with a warning,
  so you can bench one motor at a time: `joint_command` excludes them. The run stops at once if
  `joint_command` commands one anyway, which happens with an old build or a motor powered on
  mid-run;
- a moving joint is outside its limits, or would come within 2° of one;
- a move exceeds `--max-delta` (30°);
- the running `joint_command` was built with a different `hardware_mapping.yaml` or
  `safety_limits.yaml` than the repo has. The node loads its **installed** copy, so rebuild it
  (`./watod build joint_command && ./watod up -d`) after every calibration or limits edit.

`--exercise-limits` sends the request exactly as given, over exactly `--duration`: past a
joint's limits, or faster than `velocity_max`. joint_command must clamp both. Each phase then
waits for joint_command's setpoint to settle at the **clamped** target, and `summary.md` checks
that commanded stayed inside the limits and under `velocity_max`. `--max-delta` bounds the
clamped travel.

**Stopping never drops the arm:**
- Ctrl-C, or a tracking error > 8°: it ramps back to the origin.
- A second Ctrl-C: it stops streaming.
- Lost feedback: it freezes in place.

Once the stream stops, `joint_command`'s stale handling applies, as it does when teleop stops.

### Telemetry and plots

Every move — bench script or ROS pipeline — writes a **run folder** under `outputs/gl40_bench/`
(gitignored; bind-mounted into the container at `/outputs`):

```
20260922-190024_id22mp40deg/
  telemetry.csv   one row per tick per motor: t_s, motor_id, joint, phase, sp_deg, pos_deg,
                  vel_dps, tau_nm, current_a, drive_c, motor_c, status
  run.json        gains (requested + as quantised), limits, rates, git sha, abort reason
  angle.png tracking.png velocity.png torque.png summary.md
```

Logging is on by default and needs no flag (`--no-log` opts out). Plotting runs on the **host**:
the robot-control image has no matplotlib/numpy on purpose, so `uv` supplies them per run.

```bash
tools/gl40_move.sh --id 22 --deg 40 --kp 1.22 --max-track-err 12   # move + plot, one command
tools/arm_roundtrip.sh --joints wrist.pitch --offset "0,0,0,0,0,20" # out and back, joint_command
tools/gl40_ros_move.sh --pose "0,0,0,0,0,20" --duration 15         # through joint_command
uv run --with matplotlib --with numpy tools/gl40_telemetry_plot.py outputs/gl40_bench/<run>
```

Full procedure, including the clamp benchmarks and the no-hardware simulator:
[TESTING_LIMITS_AND_TELEMETRY.md](../TESTING_LIMITS_AND_TELEMETRY.md).

### No hardware? `gl40_sim.py`

Emulates all seven drives on a virtual CAN bus — five AK in servo mode (or MIT with
`--ak-mode mit`), two GL II in MIT mode,
each with inertia, damping and a gravity-like load — so the whole pipeline can be exercised
before anything is plugged in:

```bash
sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0   # once (NET_ADMIN)
python3 $S/gl40_sim.py --iface vcan0
ros2 launch can can.launch.py --ros-args -p can_interface:=vcan0 -p bustype:=socketcan
```

---

## Open arm tasks (onboarding / assignable)

Live joint mirror, mjlab sim parity, and interactive calibration are done — see
[ARM_BRINGUP.md](../../../ARM_BRINGUP.md) for calibrate → visualize → move.

| Status | Task | Why |
|--------|------|-----|
| TODO | **VR teleop** — Quest → real motors via teleop + `joint_command` / CAN | End-to-end teleop UX |
| TODO (later) | **Isaac Lab sim-to-real** — `task_space_ik.py --publish-real-left-arm` (IK) + `reach` RL task driving the real arm | Validate IK/policy against real hardware |

---

## Topics / config

`/interfacing/motorCMD` (`MotorCmd`, ROS→CAN) · `/interfacing/motorFeedback` (`MotorFeedback`, CAN→ROS)

`config/params.yaml` defaults: `can_interface=can0` `device_path=/dev/canable` `bustype=slcan` `bitrate=1000000`

DBC: `src/interfacing/dbc/humanoid.dbc` · Debug: `candump can0`
