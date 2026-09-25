# MIT clamp test: zero, move, hold, return

Step-by-step procedure for proving that **joint-angle and velocity clamping** hold on every motor.
The per-motor loop is:
1. Zero each joint at a pose you set by hand.
2. Request a pose over a chosen duration, including requests past the joint's range or faster
   than its speed limit.
3. Watch it move, hold, and come back.
4. Read the plots.

Everything runs through the same ROS path teleop uses:
`ArmPose` on `/arm/joint_targets` → `joint_command` (clamp, velocity limit, MIT gains, watchdog)
→ `can_node` → CAN. What passes here is what teleop gets.

| Motor | Joint | Drive | Path in this test |
|---|---|---|---|
| 14, 12 | shoulder pitch, roll | AK10-9 | joint_command (MIT once brought up, §3) |
| 11, 10, 13 | shoulder yaw, elbow pitch, roll | AK80-9 | joint_command (MIT once brought up, §3) |
| 22 | wrist pitch | GL40 | joint_command, MIT |
| 21 | gripper | GL40 | bench tool only (§9): it has no `ArmPose` slot |

Background: [TESTING_LIMITS_AND_TELEMETRY.md](TESTING_LIMITS_AND_TELEMETRY.md) ·
[can/README.md](can/README.md) · [joint_command/JointCommand.md](joint_command/JointCommand.md)

---

## 0. Safety: every session

- **A hardware E-stop on the 48 V motor supply, with one person's hand on it.** The software stops
  below are a complement to it, not a replacement.
- Arm base clamped; screws, brackets and cable routing checked; workspace clear.
- Nothing else publishing `/arm/joint_targets`: no teleop, `gl40_ros_move.sh` or `ros2 topic pub`.
  The round-trip tool refuses to run if something is.
- AK80-9s (10, 11, 13) **lose their zero on every power cycle**. Recalibrate them (§4) after every
  power-on.

What each kind of stop does:

| Event | What the arm does |
|---|---|
| Ctrl-C (once) | ramps back to where it started, from where it is now |
| Ctrl-C (twice) | stops streaming immediately; joints stay where they are |
| Feedback lost > 0.5 s | freezes in place; support the arm, then Ctrl-C |
| MIT fault in joint_command (torque / tracking / timeout) | GL40 goes limp, MIT AKs damp (sink slowly); restart joint_command after checking |

---

## 1. One-time setup

Run on the host, from the repo root.

```bash
# 1a. CANable on slcan firmware: lsusb must show 16d0:117e
./src/interfacing/can/scripts/can_udev.sh install      # creates /dev/canable (once per machine)

# 1b. Build the images so they carry the current C++ and scripts, then start everything
./watod build interfacing joint_command
./watod up -d

# 1c. can0 is up, and feedback is flowing
ip -details link show can0 | grep "can state"          # ERROR-ACTIVE
./watod -t interfacing
source /opt/watonomous/setup.bash
ros2 topic hz /interfacing/motorFeedback               # AK servo drives stream; GL/MIT only when polled
```

`joint_command` reads its config from the repo: `joint_command/config/*.yaml` is mounted over the
node's installed copy. A calibration or a limits edit therefore takes effect on the **next node
restart**; no rebuild is needed.

## 2. Rehearse on the simulator (optional, recommended)

The same tools against seven simulated drives on a virtual bus, so nothing can be damaged. The
isolated `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` keep it off the real stack:

```bash
C=$(docker ps -qf name=-interfacing-); E="-e ROS_DOMAIN_ID=87 -e ROS_LOCALHOST_ONLY=1"
SRC='source /opt/ros/humble/setup.bash; source /opt/watonomous/setup.bash; source /root/ament_ws/install/setup.bash'
S=/root/ament_ws/src/interfacing/can/scripts
docker exec -u root $C bash -c 'source /opt/ros/humble/setup.bash; source /opt/watonomous/setup.bash;
  cd /root/ament_ws && colcon build --packages-select can joint_command'
docker exec -u root $C bash -c 'ip link add dev vcan0 type vcan 2>/dev/null; ip link set up vcan0'
docker exec -d -u root $C bash -c "python3 $S/gl40_sim.py --iface vcan0 > /tmp/sim.log 2>&1"  # --ak-mode mit for MIT AKs
docker exec -d -u root $E $C bash -c "$SRC; ros2 run can can_node --ros-args -r __node:=can_node_vcan \
  --params-file /root/ament_ws/install/can/share/can/config/params.yaml -p can_interface:=vcan0 -p bustype:=socketcan"
docker exec -d -u root $E $C bash -c "$SRC; ros2 launch joint_command joint_command.launch.py"
docker exec -it -u root $E $C bash -c "$SRC; cd $S; python3 arm_roundtrip.py --exercise-limits \
  --joints elbow.roll --pose 0,0,0,0,200,0 --duration 2 --max-delta 60"
# afterwards: pkill -INT the three processes above, then: ip link delete vcan0
```

## 3. Put the joints you will test on MIT

- **Wrist (22):** already MIT (`safety_limits.yaml` → `wrist.pitch: control_type: 0`).
- **AKs:** one joint at a time, lightest load first: **13 → 10 → 11 → 12 → 14**. An AK you
  haven't brought up yet stays on POSITION_LOOP. It is still bound by the same angle clamp and
  velocity limit, so §5–§8 still apply to it.

For each AK:

1. **In the CubeMars upper computer:** switch the drive to MIT mode, and set its CAN timeout to
   100–200 ms.
2. **Capture its MIT feedback.** Stop joint_command, then run `candump -n 20 can0` while §4's
   calibration polls the drive. Send the capture to whoever maintains `can`: it pins the frame
   layout in `can/test/test_mit_protocol.cpp` (`DecodesAkLayoutFromTheManual` is still a
   synthetic frame from the manual).
3. **Uncomment that joint's six MIT lines** in `joint_command/config/safety_limits.yaml`:
   `control_type: 0`, `mit_family: ak`, `mit_kp`, `mit_kd`, `mit_max_torque`, `mit_fault_kd`. The
   node refuses to start if the gains break the stall rule (`kp × 12° ≤ max torque`).
4. **Check the damped fault with the arm supported.** Start joint_command, then hold the joint
   for 30 s on MIT: `tools/arm_roundtrip.sh --exercise-limits --joints <joint> --offset <2° on
   that joint> --duration 2 --dwell 30`. Then stop the node: the joint must **sink slowly**
   under damping, not drop.

## 4. Zero each joint at your pose, and record its range

This uses a software zero: `zero_offset` is written to `hardware_mapping.yaml` and nothing is
written to the drives. MIT drives are polled with zero-torque frames, so they report, **and they
are limp while this runs**. Support the arm.

```bash
# stop joint_command first (Ctrl-C in its terminal): calibrate_arm refuses to run beside it
./watod -t interfacing
source /opt/watonomous/setup.bash
sudo -E bash -c 'source /opt/watonomous/setup.bash && python3 \
  /root/ament_ws/src/interfacing/can/scripts/calibrate_arm.py \
  --arm-side left --arm-only --write-mapping --mapping /calibration/hardware_mapping.yaml'
```

`sudo -E` keeps the container's DDS settings; plain `sudo` drops them and the script never sees
`can_node`. Drop `--arm-only` to include the gripper.

For each joint the script prompts in turn:
1. **Motor id:** press Enter to confirm, or type the correct id.
2. **Zero:** hand-rotate the joint to the pose you want as 0°, then press Enter.
3. **One end:** move it to one end of its safe range, then press Enter.
4. **Other end:** move it to the other end, then press Enter.

The limits are stored 2° inside the ends you showed (`--limit-pad`); a range end is where the
clamp will stop the joint. Once every joint is done:

```bash
./watod -t joint_command
source /opt/watonomous/setup.bash
ros2 launch joint_command joint_command.launch.py
# log: "seeded from feedback", and NO "EXCLUDING joint" (see §11 if there is)
```

## 5. In-range move, one joint

`--pose` is six angles in degrees from your zero, in the order
`shoulder pitch, shoulder roll, shoulder yaw, elbow pitch, elbow roll, wrist pitch`. `--joints`
picks which of them move; the rest hold where they are.

```bash
tools/arm_roundtrip.sh --exercise-limits --joints elbow.roll --pose "0,0,0,0,10,0" \
  --duration 3 --dwell 3
```

The run goes through:
1. **hold** 2 s at the start, checking that joint_command commands the measured pose;
2. **ramp** over `--duration` s;
3. **dwell** for `--dwell` s;
4. **return** over `--duration` s;
5. **rest**.

Expect the commanded trace to equal the requested one, and "at target, error" within a degree or
two. GL40s sag a few degrees under PD, which is normal.

## 6. Angle clamp: request past the limit

```bash
tools/arm_roundtrip.sh --exercise-limits --joints elbow.roll --pose "0,0,0,0,200,0" \
  --duration 2 --dwell 3 --max-delta 60
```

The script prints `request +200 is past its limits ... should clamp it to <upper>`. Expect:
- **`angle.png`:** the dotted *requested* trace leaves the top of the panel, while *commanded*
  and *measured* stop on the dashed limit, hold, and come back.
- **`summary.md`:** `clamped_to_deg` equals the limit, and PASS on
  `commanded stayed inside joint limits`.

`--max-delta` (default 30°) bounds the **clamped** travel. Raise it only as far as the joint's
range from zero needs. Repeat with a large negative request for the lower limit.

## 7. Velocity clamp: request faster than `velocity_max`

```bash
tools/arm_roundtrip.sh --exercise-limits --joints elbow.roll --pose "0,0,0,0,20,0" \
  --duration 0.5 --dwell 3
```

The table printed at the start shows the request's peak (about 63 °/s here) against
`vel max` (10 °/s). Expect:
- `summary.md`: `peak_requested_vel_dps` ≫ 10,
  `peak_commanded_vel_dps` ≤ 10.5, `velocity_clamp_exercised: True`, and PASS on
  `commanded velocity <= velocity_max`;
- `velocity.png` stays under its dashed ceiling.

## 8. All joints

Repeat §5–§7 for each joint (`--joints shoulder.yaw`, …), then all six at once:

```bash
tools/arm_roundtrip.sh --exercise-limits --pose "5,2,10,10,10,15" --duration 3              # in range (fit to your ranges)
tools/arm_roundtrip.sh --exercise-limits --pose "200,200,200,200,200,200" --duration 2 --max-delta 60   # every clamp at once
```

## 9. Gripper (21): bench tool, bypasses ROS

The gripper has no `ArmPose` slot, so joint_command can't clamp it. The bench tool loads the same
yaml limits: angle range, `velocity_max`, and torque and tracking ceilings. Flags may only
tighten them, and the joint has no validated gains, so pass `--kp` and `--kd`. Stop
joint_command first.

```bash
tools/gl40_move.sh --id 21 --deg 20 --kp 1.22 --kd 0.0098 --joint gripper.open_close
tools/gl40_move.sh --id 21 --deg 200 --kp 1.22 --kd 0.0098 --clamp-target   # clamps to its range, then returns
```

## 10. Reading the output

Each run writes `outputs/gl40_bench/<UTC time>_<label>/`, and the wrapper plots it on the host:

| File | Look for |
|---|---|
| `angle.png` | requested (dotted), commanded (dashed), measured, joint limits; phase bands hold/ramp/dwell/return/rest |
| `velocity.png` | measured velocity under the dashed `velocity_max` ceiling |
| `tracking.png` | \|commanded − measured\| under the fault threshold |
| `summary.md` | PASS/FAIL per check per motor; `requested_deg`, `clamped_to_deg`, `benchmark_deg` (where it held), `steady_state_err_deg` (sag), `return_err_deg` (how close it came home), `peak_requested_vel_dps` / `peak_commanded_vel_dps` |
| `run.json` | origin / requested / expected pose, limits, gains, outcome |

To re-plot a run by hand:

```bash
uv run --with matplotlib --with numpy tools/gl40_telemetry_plot.py outputs/gl40_bench/<run>
```

What a FAIL means:
- **`commanded stayed inside joint limits`:** the clamp let a command through. Stop and
  report it.
- **`commanded velocity <= velocity_max`:** the limiter let a command through. Stop and
  report it.
- **`measured stayed inside joint limits`** with the commanded check passing: overshoot or sag
  beyond 2°. Check the gains.
- **`peak tracking error`:** the joint couldn't follow. Check for load, gains, or a mechanical
  bind.

Keep every run folder: it is the evidence that the limits held.

## 11. Troubleshooting

| Message / symptom | Cause, and what to do |
|---|---|
| `did not follow: <joint> moved +0.0 of ...` | The drive isn't enabled, powered or commanded. Check the joint_command log for `EXCLUDING` / MIT FAULT, and that the drive is in the mode §3 set. |
| `EXCLUDING joint ...` (joint_command log) | The joint sits outside its own limits. Redo §4 for it, then restart joint_command. |
| `joint_command_node is enforcing an older config` | The config mount isn't active. Run `./watod up -d` to recreate the container, or rebuild joint_command. |
| `something else is publishing /arm/joint_targets` | Stop the teleop / publisher it names. |
| `no feedback from ...` | The drive is unpowered or has the wrong id, or `can_node` is down. Check `candump can0` and `ros2 topic echo /interfacing/motorFeedback`. |
| `can0` missing after re-plugging the CANable | slcand died with the old tty. Run `docker restart $(docker ps -qf name=-interfacing-)`. |
| calibrate_arm: `No feedback for motor_id=…` on a GL40/MIT AK | joint_command is still running, or the id is wrong. The drive only answers when polled. |
| MIT FAULT in joint_command | It latches. Find the cause in the log (torque, tracking or timeout), then restart the node. |
