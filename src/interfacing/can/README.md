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

### GL40 II in MIT mode — bench only (`gl40_mit_move.py`)

The wrist/gripper GL40s use CubeMars' **GL II gimbal drive**, which in MIT mode speaks
*standard* 11-bit frames (`ID = node id`, feedback on the master id, default `0x000`) with the
canonical `pos(16) vel(12) kp(12) kd(12) t_ff(12)` byte order
([manual §5](https://www.cubemars.com/images/file/20241231/1735633965678815.pdf)).
`can_node`'s `MIT_CONTROL` cannot drive it yet: `humanoid.dbc` packs `kp,kd,pos,vel,t`, every TX
frame is sent as an extended id, there is no GL40 entry in `config/mit_profiles.yaml`, no
enter/exit-motor-mode frames, and MIT feedback is not decoded. Until that is fixed, use the
raw-SocketCAN bench script (stdlib only, no ROS; needs `can0` up, i.e. `can_node` running or
`setup_can.sh`). Make sure `joint_command_node` is **not** running.

```bash
S=/root/ament_ws/src/interfacing/can/scripts/gl40_mit_move.py   # /root/ament_ws is root-only in the container -> sudo
sudo python3 $S --selftest             # frame packing vs. manual example (no bus)
sudo python3 $S --id 22 --monitor      # zero torque; turn the shaft by hand, check the rad scale
sudo python3 $S --id 22 --deg 40 --dry-run  # print frames only
sudo python3 $S --id 22 --deg 40       # +40° from the current position, then go limp (--hold to stay)
```

It reads the position first, holds it for 1 s, ramps the setpoint (≤ 1 rad/s, default 40° in 4 s),
and frees the motor on Ctrl-C or on any abort (torque > 0.3 N·m, tracking error > 15°, > 60 °C,
drive error, 200 ms without feedback). Gains are snapped to the drive's 12-bit codes (kp 0.122,
kd 0.0012 N·m per count) and must satisfy `kp × max-track-err ≤ max-torque` (a stalled motor is the
worst case: the tracking abort caps the PD torque). Defaults **kp raw 3 = 0.366 N·m/rad, kd raw 8 =
0.0098 N·m·s/rad** (GL40 KV70 rated 0.25 / peak 0.73 N·m; the manual's own example is 0.123 / 0.005;
kd must be non-zero). Bench result 2026-09-19, id 22: the shaft had a ~0.05 N·m restoring load, so
kp 0.366 lagged >15° and aborted; kp 0.49 held with a 6° sag; `--kp 1.22 --max-track-err 12 --hold`
(worst case 0.26 N·m) moved +34° and held steady at 0.125 N·m with a 5.8° sag — pure PD under the
0.3 N·m ceiling cannot do better against that load; use `--hold`, since the shaft falls back to its
rest position as soon as the motor is freed, and add torque feed-forward or the drive's
position-velocity mode if zero steady-state error is needed. `--p-max/--v-max/--t-max` must match the
drive's parameter page (defaults ±12.5 rad / ±200 / ±10 N·m) — `--monitor` is the way to check `--p-max`.

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
