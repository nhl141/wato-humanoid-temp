# Joint command: `joint_command_node` + `joint_command_core`

We convert high-level arm joint targets (`ArmPose`) into per-motor CAN commands (`MotorCmd`), with YAML-driven calibration and runtime safety moderation (clamp, rate limit, smoothing). Intended for policy / teleop outputs before hardware.

## Pipeline

**Input:** `common_msgs/ArmPose` on `/arm/joint_targets` (6 angles: 3 shoulder, 2 elbow, 1 wrist).

**Output:** six `common_msgs/MotorCmd` messages on `/interfacing/motorCMD` (`POSITION_LOOP` by default).

**Node behavior:**
1. Each `ArmPose` is **cached**, not acted on.
2. A timer at `control_rate_hz` runs the moderation pipeline one step and publishes. Moderation
   therefore advances per **tick**, not per message: `velocity_max` is a true degrees-per-second
   bound no matter how fast the publisher sends, and one `ArmPose` is enough for the arm to ramp
   all the way to it. (It used to advance once per message — a single pose moved one step and
   stopped, and streaming at 200 Hz moved the arm 4× faster than at 50 Hz.)
3. Nothing is published until the rate-limiter has been **seeded from real feedback**, so motion
   always ramps from where the arm actually is.
4. If no `ArmPose` arrives within `command_timeout_sec`, commands stop and the next one must
   re-seed (MIT joints are left limp).

## Per-joint processing (`armPoseToMotorCmds`)

For each joint $i$, let $q^{\mathrm{in}}_i$ be the incoming angle (degrees, same units as
`hardware_mapping.yaml`), applied once per control tick:

1. **Position clamp** — clip to the hardware limits: $q \leftarrow \mathrm{clip}(q, q_{\min}, q_{\max})$.
2. **Low-pass** — exponential smoothing with $\alpha =$ `low_pass_alpha`:
   $q \leftarrow \alpha q^{\mathrm{prev}} + (1-\alpha) q$.
3. **Velocity limit** — cap the change this tick: $\Delta q_{\max} = \texttt{velocity\_max} / \texttt{control\_rate\_hz}$.
4. **Delta limit** — additional per-tick cap `delta_max`.
5. **Position clamp again**, *itself rate-limited*. Clamping last would otherwise undo steps 2–4:
   if the joint is currently outside its limits (stale calibration, placeholder limits,
   hand-moved arm), an unbounded clamp snaps it to the limit in one tick — ~195° on this arm's
   mapping. So when the clamp bites, the move back into range is limited to the same per-tick
   budget. When the previous target is already in range this is a no-op.
6. **Calibration** — $q_{\mathrm{motor}} = \texttt{direction} \cdot (q - \texttt{zero\_offset})$.

The low-pass runs **before** the rate clamps. The other way round it shrank each step the
velocity limiter had just sized, making the real top speed $(1-\alpha)\cdot$`velocity_max` — 6°/s
where the config said 40.

## Per-joint control type and MIT joints

`control_type` may be set per joint in `safety_limits.yaml` (`-1` = the node default from
`joint_command.yaml`). The GL40 wrist runs `MIT_CONTROL` (0); the AK joints ship on `POSITION_LOOP`
with commented-out MIT blocks — see "AK joints in MIT mode" in [can/README.md](../can/README.md)
before enabling one.

A MIT joint is a PD drive with **no internal limit checking**, so `joint_command` supplies:

| Field | Role |
|---|---|
| `mit_kp` / `mit_kd` | stiffness / damping, physical units, sent in `MotorCmd` |
| `mit_max_torque` | fault above this reported torque |
| `mit_max_track_err` | fault if the joint lags its setpoint by more |
| `mit_feedback_timeout` | fault after this long without feedback |
| `mit_family` | `gl2` (GL40) or `ak` (AK10-9 / AK80-9) — they disagree about the status byte: on an AK, 1 is **motor over-temperature**, on a GL II it is "Enable" |
| `mit_fault_action` | `limp` (kp = kd = 0, then `MIT_EXIT`) or `damp` (kp = 0, kd = `mit_fault_kd`, streamed). Defaults to `damp` for `ak`, `limp` for `gl2` |
| `mit_fault_kd` | damping used by `damp`; must be in (0, 5] N·m·s/rad or the node refuses to start |

**The gain rule, enforced at startup** (the node refuses to launch otherwise):
quantised `mit_kp` × `mit_max_track_err` (rad) ≤ `mit_max_torque`. The worst case is a stalled
joint: the tracking fault fires at `mit_max_track_err`, so PD torque can never exceed that
product. Gains are quantised to the drive's 12-bit code (kp: 500/4096 per count) before the
check, so it tests what the motor actually gets.

Lifecycle: `MIT_ENTER` on startup (a GL II ignores commands until entered) → **safe frames**
(kp = 0; kd = 0 for `limp`, `mit_fault_kd` for `damp`) while unseeded, since drives only answer
when spoken to → gains applied once seeded → safe frames again on a stale command stream.

On a **fault**, `limp` joints get `MIT_EXIT` and `damp` joints keep receiving damping frames every
tick. Stopping the stream would trip an AK's own CAN timeout, which cuts output and drops a
gravity-loaded arm just as surely as going limp. A fault **latches**: nothing but those frames is
published until the node restarts.

On **Ctrl-C**, `damp` joints are damped for `mit_shutdown_damp_sec` (param, default 2 s), then every
MIT joint gets `MIT_EXIT`. This runs from a pre-shutdown callback: after `rclcpp::spin` returns,
the context is already invalid and `publish()` silently drops messages. The earlier post-spin
free never reached the bus. It still needs `can_node` alive to forward the frames, so stop
`joint_command` **before** `can_node`, and keep a loaded arm supported.

## Joints found outside their own limits

If seeding shows a joint physically outside its configured limits, its calibration and the
hardware disagree. That joint is **excluded** (MIT ones get their fault action) with an error naming it,
rather than silently clamped — clamping would walk it to the limit the moment it is commanded.
The rest of the arm still works. Fix by re-running `calibrate_arm.py`.

## Joints with no feedback (unpowered)

A joint whose motor sent no feedback in the last 0.5 s when seeding happens is **excluded** until
the next seed. Servo joints get no command, and MIT joints get their kp = 0 fault action. It is
never ramped from an assumed 0, because a drive powered on mid-session would jump to that stream.
So you can bench one motor at a time. A motor powered on mid-stream stays excluded until the
`ArmPose` stream goes stale and re-seeds, for example between two `arm_roundtrip.sh` runs. One
silent MIT joint no longer stops the other joints from being commanded.

## Config files

| File | Role |
|------|------|
| `config/joint_command.yaml` | ROS params: arm side, topics, control rate, control type |
| `config/hardware_mapping.yaml` | Per-joint `can_id`, limits, `direction`, `zero_offset` |
| `config/safety_limits.yaml` | Moderation toggles, per-joint `velocity_max`/`delta_max`/`low_pass_alpha`, per-joint `control_type`, and the MIT gains + fault thresholds |

Safety YAML uses a top-level `safety:` key with `global` defaults and optional `joints` overrides (shoulder/elbow/wrist paths match hardware mapping).

## Tuning `safety_limits.yaml`

Units are **degrees** and **deg/s**. `velocity_max` is now a genuine deg/s bound (see above), so
at 50 Hz `velocity_max: 10` means 0.2°/tick and 10°/s of actual arm motion.

Start conservative on hardware, then increase until motion is responsive without jitter or limit
hitting. Current values are a deliberate crawl (10°/s), matching what the arm really did before
the ordering fix.

| Parameter | Effect |
|-----------|--------|
| `velocity_max` | Max joint speed, deg/s (converted to °/tick internally) |
| `delta_max` | Hard cap on ° change per tick, independent of the rate |
| `low_pass_alpha` | Higher → smoother/slower approach (e.g. `0.85`) |
| `enable_*` | Toggle each stage without recompiling |
| `control_type` | Per joint; `0` = MIT_CONTROL, `4` = POSITION_LOOP, `-1` = node default |
| `mit_*` | Gains and fault thresholds for MIT joints (see the gain rule above) |

## Tests

`colcon test --packages-select joint_command` runs gtests against the **shipped** config, so a
change that disables the position clamp, raises a velocity past the 2 rad/s testing ceiling, or
breaks the MIT gain rule fails the build rather than the arm. See
[TESTING_LIMITS_AND_TELEMETRY.md](../TESTING_LIMITS_AND_TELEMETRY.md) for running them and for
the on-hardware benchmarks.

## Launch

```bash
ros2 launch joint_command joint_command.launch.py
```

**Defaults:** `arm_side=left`, `control_rate_hz=50`, `control_type=POSITION_LOOP` (4),
overridden per joint in `safety_limits.yaml` (the wrist runs MIT).
