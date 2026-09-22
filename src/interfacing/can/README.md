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

The raw-SocketCAN bench tools remain, for one motor at a time without ROS in the way:

```bash
S=/root/ament_ws/src/interfacing/can/scripts   # /root/ament_ws is root-only -> sudo
sudo python3 $S/gl40_mit_move.py --selftest                 # packing vs the manual (no bus)
sudo python3 $S/gl40_mit_move.py --id 22 --monitor          # zero torque; check the rad scale
sudo python3 $S/gl40_mit_move.py --id 22 --deg 40 --dry-run # print frames only
sudo python3 $S/gl40_mit_move.py --id 22 --deg 40 --kp 1.22 --max-track-err 12 --hold
sudo python3 $S/gl40_bench.py    --id 22 --step 5 --sweep "0.61,1.22,1.34"   # gain sweep
```

It reads the position first, holds it, ramps the setpoint (≤ 1 rad/s), and frees the motor on
Ctrl-C or on any abort: torque > 0.3 N·m, tracking error > 15°, shaft velocity > 3 rad/s,
outside `--soft-limits`, > 60 °C, drive error, or 200 ms without feedback. `--soft-limits
LO,HI` refuses an out-of-range target (or clamps it with `--clamp-target`, which is how limit
enforcement is demonstrated). Gains must satisfy `kp × max-track-err ≤ max-torque` — a stalled
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
tools/gl40_ros_move.sh --pose "0,0,0,0,0,20" --duration 15         # through joint_command
uv run --with matplotlib --with numpy tools/gl40_telemetry_plot.py outputs/gl40_bench/<run>
```

Full procedure, including the clamp benchmarks and the no-hardware simulator:
[TESTING_LIMITS_AND_TELEMETRY.md](../TESTING_LIMITS_AND_TELEMETRY.md).

### No hardware? `gl40_sim.py`

Emulates all seven drives on a virtual CAN bus — five AK in servo mode, two GL II in MIT mode,
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
