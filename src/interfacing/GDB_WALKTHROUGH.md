# Stepping through the arm pipeline with gdb

A hands-on guide for someone who has never used gdb. By the end you will have followed one
command from `arm_roundtrip.py` through `joint_command_node` and `can_node` to the exact 8 bytes
that go on the CAN bus, and followed the motor's reply back. At each step you will know which
motor-manual table that step depends on.

Everything here runs against **`gl40_sim.py` on a virtual CAN bus (`vcan0`)**, never the real
arm. Every command and every output block below was captured from a real run on the
`gl40-interface` branch (commit `d82240a`). Line numbers are from that commit. If the code has
moved since, use the function name instead (`break JointCommandCore::armPoseToMotorCmds`), or
`list <function>` to find the new line.

> ⚠️ **Never attach gdb to the pipeline while the motors are powered.** A process stopped at a
> breakpoint cannot run its safety checks. §6 covers what to do on the real arm instead.
> Keep the hardware E-stop within reach whenever the motor supply is on.

---

## 0. The three rules (the reasons are in the sections below)

1. **Use a separate ROS domain.** Every debug shell runs `export ROS_DOMAIN_ID=42`. The
   interfacing container's real `can_node` (the one talking to `can0`) is on domain 0. If your
   debug `joint_command` is on domain 0 too, its commands reach the real motors whenever the
   CANable is plugged in.
2. **Stop freely only in the unit tests.** In the live pipeline, every millisecond a node spends
   stopped counts against a watchdog (§4.3).
3. **Never put a breakpoint with a condition on a line that runs every tick.** gdb has to stop
   the whole process each time just to evaluate the condition. Measured: two such breakpoints cut
   joint_command from **250 to 4.2 MotorCmd/s**, and the wrist stopped following. Stop with
   Ctrl-C, then set a one-shot `tbreak` (§4.4).

---

## 1. gdb in ten minutes

### The ideas

| term | meaning |
|---|---|
| **process** | one running program. The pipeline has three: `arm_roundtrip.py` (Python), `joint_command_node` (C++), `can_node` (C++). Plus `gl40_sim.py`, which stands in for the motors. |
| **thread** | a process has several. Your code runs on thread 1 (the ROS executor); the rest belong to DDS. When gdb stops a process, **all** of its threads stop. |
| **breakpoint** | "stop when execution reaches this line or function" |
| **frame / call stack** | the chain of function calls that led to the current line. `bt` prints it; frame #0 is where you are. |
| **step vs next** | `next` runs the current line and stops on the next one *in this function*. `step` goes *into* any function called on this line. `finish` runs until the current function returns and prints what it returned. |

**There is no single call stack across the pipeline.** Each process has its own, and they are
joined by ROS topics. You follow a command by stopping in one process, noting a value (for
example a position of `-45.84`), then finding the same value in the next process.

### Cheat sheet

| gdb | pdb (Python) | what it does |
|---|---|---|
| `run` / `r` | (starts automatically) | start the program |
| `break file.cpp:123` / `b Func` | `b 123` / `b func` | set a breakpoint |
| `tbreak ...` | `tbreak ...` | one-shot breakpoint (deletes itself after the first hit) |
| `info breakpoints` / `delete 3` | `b` / `cl 3` | list / remove breakpoints |
| `continue` / `c` | `c` | run until the next breakpoint |
| `next` / `n` | `n` | next line, don't go into calls |
| `step` / `s` | `s` | next line, go into calls |
| `finish` | `r` | run to the end of this function |
| `until 538` | `until 538` | run until that line (handy for skipping a loop) |
| `print x` / `p x` | `p x` / `pp x` | print a variable. gdb also takes `p/x x` (hex) and `p a - b` (expressions) |
| `info locals` / `info args` | `a` | all local variables / the function's arguments |
| `bt` / `bt 5` | `w` | call stack (all frames / 5 innermost) |
| `frame 2` / `up` / `down` | `u` / `d` | move up/down the stack to inspect a caller's variables |
| `list` | `l` | show source around the current line |
| `x/8xb ptr` | — | dump 8 bytes of raw memory in hex |
| `set var x = 1` | `!x = 1` | change a variable in the running program |
| Ctrl-C | Ctrl-C | stop the program right now, wherever it is |
| `quit` | `q` | exit (kills the program) |

Two traps for beginners:
- `step` into a line like `v[i] = x` goes into the C++ standard library (`stl_vector.h`). Type
  `finish` to get back out, or avoid it permanently with `skip -gfi /usr/include/*` (in
  `~/.gdbinit` below).
- Pressing Enter on an empty line repeats the last command. That's handy for `next`, `next`,
  `next`.

### `~/.gdbinit` (create once per container; gdb reads it at startup)

```gdb
set pagination off
set print pretty on
set print frame-arguments none
set print frame-info short-location
skip -gfi /usr/include/*
```

The two `frame-*` lines matter: without them a ROS backtrace prints the full templated type of
every `std::function` and fills several screens per frame.

---

## 2. Setup (once, until the containers are recreated)

This takes four terminals. `watod -t <service>` opens a shell in a service container. Both
interfacing containers use host networking, so they share the host's `vcan0`.

```bash
# host, once per boot: the virtual CAN bus
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null; sudo ip link set up vcan0
```

In **each** of the four shells below, start with:

```bash
source /opt/watonomous/setup.bash
export ROS_DOMAIN_ID=42          # rule 1 -- never skip this
```

Then check that you're isolated before going further:

```bash
echo $ROS_DOMAIN_ID              # must print 42
ros2 node list                   # must NOT list the container's own can_node (that one is on domain 0)
```

### Install gdb and build Debug copies (once per container)

The images are built `Release` (optimised, no debug info: gdb can't show variables) and do not
include gdb. Also, the source is bind-mounted under `/root`, which your container user can't
enter. These fixes are local to the container and are lost when it is recreated:

```bash
# in BOTH ./watod -t interfacing and ./watod -t joint_command
sudo chmod o+x /root
sudo apt-get update && sudo apt-get install -y gdb
```

```bash
# ./watod -t interfacing   -> Debug can_node (+ its unit tests)
mkdir -p ~/dbg_ws && cd ~/dbg_ws
colcon build --symlink-install --base-paths /root/ament_ws/src/interfacing/can \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug

# ./watod -t joint_command -> Debug joint_command_node (+ its unit tests)
mkdir -p ~/dbg_ws && cd ~/dbg_ws
colcon build --symlink-install --base-paths /root/ament_ws/src/joint_command \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

`--symlink-install` makes the debug node read the **repo's** YAML files, which are the same
limits the real node enforces. The `Release` binaries in `/opt/watonomous` are untouched.
Rebuild after editing C++; YAML changes need only a node restart.

You will see `warning: Error disabling address space randomization: Operation not permitted`
every time gdb starts a program in these containers. It's harmless.

---

## 3. Lesson 1: step through the limit logic with no timing at all

The unit tests call the same `JointCommandCore` and `mit_protocol` code the nodes run, loaded
with the **shipped** config, but no ROS, no bus and no watchdogs. You can stop for as long as
you like. Start here.

### 3.1 How one tick of `armPoseToMotorCmds` turns a request into a motor command

```bash
# ./watod -t joint_command
cd ~/dbg_ws
build/joint_command/test_joint_command_core --gtest_list_tests    # what's available
gdb --args build/joint_command/test_joint_command_core \
  --gtest_filter=ShippedConfig.FirstCommandRampsFromTheMeasuredPoseNotFromZero
```

In this test the arm is parked at +2° (command frame) and the request is 0°. The first tick
must move at most one velocity step. Follow elbow pitch (joint index 3):

```
(gdb) break JointCommandCore::armPoseToMotorCmds
(gdb) run
Breakpoint 1, JointCommandCore::armPoseToMotorCmds (...) at .../joint_command_core.cpp:452
(gdb) bt 2
#0  JointCommandCore::armPoseToMotorCmds (...) at .../joint_command_core.cpp:452
#1  ShippedConfig_FirstCommandRampsFromTheMeasuredPoseNotFromZero_Test::TestBody (...) at .../test_joint_command_core.cpp:222
(gdb) p prev_targets_                       <- the seeded pose, command-frame degrees
$1 = std::vector of length 6, capacity 6 = {2, 2, 2, 2, 2, 2}
(gdb) break joint_command_core.cpp:496 if i == 3      <- OK here: a test, not a live node
(gdb) c
496	    double target = source_angles[i];
(gdb) p jointName(i)
$3 = "elbow.pitch"
(gdb) p source_angles                       <- what was asked for
$4 = {0, 0, 0, 0, 0, 0}
(gdb) n                                     <- position clamp: 0 is inside the limits
(gdb) n
(gdb) n
(gdb) p target
$7 = 0
(gdb) n                                     <- low-pass: 0.85*2 + 0.15*0
(gdb) n
(gdb) p target
$8 = 1.7
(gdb) until 538                             <- velocity limit, delta limit, final clamp
538	    next_targets[i] = target;
(gdb) p target
$10 = 1.8
(gdb) p prev_targets_[i]
$11 = 2
(gdb) p safety_[i].velocity_max / control_rate_hz_     <- the per-tick cap: 10 deg/s / 50 Hz
$12 = 0.2
(gdb) n
540	    const double calibrated_deg = applyCalibration(target, joints_[i]);
(gdb) n
(gdb) p calibrated_deg                      <- direction*(q - zero_offset) = -1*(1.8 - -105.5)
$14 = -107.3
```

The step was 2 → 1.8, exactly 0.2° = `velocity_max / rate`. The low-pass wanted 1.7, and the
velocity limit held it back. `calibrated_deg` is the **motor-frame** angle that goes out in the
MotorCmd.

Try the same with other tests: `CommandsNeverLeaveTheConfiguredJointLimits`,
`ClampHoldsAfterSmoothing`, `JointFoundOutsideItsOwnLimitsIsFlaggedAndExcluded`,
`MitWatchdogCatchesEveryFaultCondition`.

### 3.2 How physical units become MIT bytes

```bash
# ./watod -t interfacing
cd ~/dbg_ws
gdb --args build/can/test_mit_protocol --gtest_filter=MitPacking.MatchesGlManualExample
```

```
(gdb) break packMitCommand
(gdb) run
Breakpoint 1, packMitCommand (p=2, v=0, kp=0.123, kd=0.005, t=0, profile=...) at .../mit_protocol.cpp:42
(gdb) info args
profile = @0x...: {p_min = -12.5, p_max = 12.5, v_min = -200, v_max = 200, t_min = -10, t_max = 10,
                   kp_min = 0, kp_max = 500, kd_min = 0, kd_max = 5, family = MitFamily::Gl2, model = "GL40-KV70"}
(gdb) n            (x5, one per field)
(gdb) p/x p_i
$2 = 0x947a        <- 2 rad in a 16-bit field spanning -12.5..12.5
(gdb) p kp_i
$3 = 1             <- 0.123 N.m/rad is ONE count (500/4096 = 0.122 per count)
(gdb) p kd_i
$4 = 4
(gdb) p/x v_i
$5 = 0x800         <- 0 rad/s is the middle of the 12-bit range
(gdb) finish
Value returned is $7 = {_M_elems = "\224z\200\000\001\000H"}     <- 94 7A 80 00 01 00 48 00
```

This is the GL II manual's worked example. **`profile` is the key datasheet dependency**: every
number that goes to a MIT drive is scaled by these ranges (§5, row 3).

---

## 4. Lesson 2: the live pipeline, in the sim

### 4.1 What is running, and how data flows

```
 arm_roundtrip.py ──ArmPose──► joint_command_node ──MotorCmd──► can_node ──CAN frame──► vcan0 ◄──► gl40_sim.py
 (50 Hz loop)  /arm/joint_targets  (50 Hz timer)  /interfacing/motorCMD   (poll 10 ms)                (the "motors")
       ▲                                ▲                                     │
       └──────────── MotorFeedback ─────┴──── /interfacing/motorFeedback ◄────┘
```

| process | shell | units in / out |
|---|---|---|
| `gl40_sim.py` | interfacing, terminal 1 | raw CAN frames. It emulates ids 10–14 (AK, servo mode) and 21, 22 (GL40, MIT) |
| `can_node` | interfacing, terminal 2 | MotorCmd → bytes. Servo mode: **degrees** in the DBC's `PositionLoopCmd`. MIT: **radians** packed as in §3.2 |
| `joint_command_node` | joint_command, terminal 3 | ArmPose command-frame **degrees** → MotorCmd **motor frame** (deg for servo, rad for MIT) |
| `arm_roundtrip.py` | joint_command, terminal 4 | publishes ArmPose, reads feedback |

How to look at it from outside, without stopping anything (any extra domain-42 shell):

```bash
ros2 node list                                   # can_node, joint_command_node, telemetry_recorder
ros2 topic info -v /interfacing/motorCMD         # who publishes / subscribes, with QoS
ros2 topic echo --once /interfacing/motorCMD
ros2 topic hz /interfacing/motorCMD              # 250/s = 5 motors x 50 Hz. Much lower = something is stalling joint_command
candump -td -x vcan0                             # the bytes on the wire (interfacing container)
```

In `candump`, **3-hex-digit ids are standard frames (MIT)** (`016` = motor 22) and **8-digit
ids are extended frames (servo mode)** (`0000040A` = position loop to motor 10, `0000290A` =
status from motor 10).

### 4.2 Start everything

```bash
# terminal 1 (interfacing): the fake motors
python3 /root/ament_ws/src/interfacing/can/scripts/gl40_sim.py --iface vcan0

# terminal 2 (interfacing): can_node under gdb
source ~/dbg_ws/install/setup.bash
gdb --args ~/dbg_ws/install/can/lib/can/can_node --ros-args \
  --params-file ~/dbg_ws/install/can/share/can/config/params.yaml \
  -p can_interface:=vcan0 -p bustype:=socketcan
(gdb) run

# terminal 3 (joint_command): joint_command under gdb (breakpoints for 4.3 go in before `run`)
source ~/dbg_ws/install/setup.bash
gdb --args ~/dbg_ws/install/joint_command/lib/joint_command/joint_command_node --ros-args \
  --params-file ~/dbg_ws/install/joint_command/share/joint_command/config/joint_command.yaml

# terminal 4 (joint_command): the script, only once both nodes print "ready"
cd /opt/humanoid_scripts
python3 arm_roundtrip.py --joints wrist.pitch --offset "0,0,0,0,0,10" --no-log --feedback-timeout 30
```

(`--feedback-timeout 30` is for the sim only; see 4.3. On the real arm leave it at its default.)

### 4.3 Why stopping a live node trips the safety checks, and the sim-only workaround

What happened when I stopped `joint_command_node` at the seed for ~1.5 s (to print a backtrace):

```
[ERROR] [joint_command_node]: MIT FAULT: wrist.pitch (motor 22): MIT feedback is 1.50097 s old (limit 0.2 s).
        Limp MIT joints are freed ... commands are halted. Restart the node after checking the hardware.
```

and in the script: `FEEDBACK LOST from wrist.pitch: holding the current pose.`

Even the time taken to print two variables inside the seeding tick was enough (`0.231119 s old`).
And stopping *just before* the seed made the wrist's feedback look stale, so joint_command
refused to seed from it: `EXCLUDING joint wrist.pitch (motor 22): no feedback -- not commanded
(MIT: held at kp=0)`. On the real arm, **both of these are the correct response**: when the
software stops watching, the wrist goes limp. They just make interactive stepping impossible.

Why this happens: the GL40 only replies when it receives a frame. While joint_command is
stopped, no frames go out, no feedback comes back, and the wrist's 0.2 s `mit_feedback_timeout`
expires.

**Sim-only workaround:** raise that one threshold *in the running process's memory*. This
edits no file, and the change is gone when the process exits. In terminal 3, before `run`:

```
(gdb) tbreak JointCommandNode::controlTimerCallback
(gdb) run
Temporary breakpoint 1, JointCommandNode::controlTimerCallback (...)
(gdb) p core_.safety_[5].mit_feedback_timeout        <- index 5 = wrist.pitch (ArmPose order)
$1 = 0.2
(gdb) set var core_.safety_[5].mit_feedback_timeout = 30.0
(gdb) c
```

> ⚠️ This is only for gdb sessions against `gl40_sim`. Never do it to a node that can reach real
> motors: while it's raised, a stuck wrist goes unnoticed for 30 s. Never make the same change
> in `safety_limits.yaml`.

With this in place I stopped joint_command for 3 s in the middle of a ramp. There was no fault,
and the run finished normally. It was flagged "did not follow" only because the node was frozen
for most of the ramp, which is why a run you stopped doesn't count as a benchmark result.

### 4.4 Stopping mid-motion correctly: Ctrl-C, then a one-shot `tbreak`

Don't write `break joint_command_core.cpp:541 if i == 5` in a live node. It runs 300 times a
second (6 joints × 50 Hz), and gdb stops every thread to test the condition each time (rule 3).
Instead:

1. Wait for the script to print `[ramp]`.
2. Press **Ctrl-C in terminal 3**. gdb stops joint_command wherever it is, usually asleep in the
   executor:
   ```
   #0  __futex_abstimed_wait_common64 (...)
   ```
3. Set a **one-shot** breakpoint for the wrist line and continue. The condition is tested only
   until the first hit, which is less than one tick away:
   ```
   (gdb) tbreak joint_command_core.cpp:541 if i == 5
   (gdb) c
   Thread 1 "joint_command_n" hit Temporary breakpoint 2, JointCommandCore::armPoseToMotorCmds (...)
   (gdb) p source_angles[i]        <- what the script asked for this tick
   $1 = -45.365121684838854
   (gdb) p prev_targets_[i]        <- last tick's output
   $2 = -45.927150038036693
   (gdb) p target                  <- this tick's output, after every limiter
   $3 = -45.842845785057015
   (gdb) p target - prev_targets_[i]
   $4 = 0.084304252979677585       <- the step taken this tick
   (gdb) p safety_[i].velocity_max / control_rate_hz_
   $5 = 0.59999999999999998        <- the most it may be: 30 deg/s / 50 Hz
   (gdb) p calibrated_deg          <- motor frame; goes out as 47.497 deg = 0.829 rad
   $6 = 47.49721158798986
   (gdb) info locals               <- everything else, including the MotorCmds built so far
   (gdb) c
   ```

This is the pattern to use anywhere in the live pipeline: **Ctrl-C, then `tbreak`, then
inspect, then `c`**.

### 4.5 The stops worth making, in the order a command travels

Set these as `tbreak` (Ctrl-C first if the node is already running). Links go to the source.

**Script: [arm_roundtrip.py](can/scripts/arm_roundtrip.py)** (pdb, terminal 4:
`python3 -m pdb arm_roundtrip.py ...`)

| where | look at | why it matters |
|---|---|---|
| `b 331` (target computation) | `p name, origin[i]`, `n`, `p target[i]` | Origin comes from **measured** feedback; the target is origin + offset. Real run: `('wrist.pitch', -50.90)` → `-40.90`. |
| `b 535` (seed check) | `p infos[i]['name'], sp, pos` | The script refuses to move if joint_command's first setpoint differs from the measured pose by more than 2°. |
| `b 98` (publish) | `p pose` | The exact ArmPose going out this tick. |

Pausing the script is harmless to the nodes (joint_command just holds the last pose). The
script's own feedback watchdog fires when you continue, so that run ends early; start a new one.
In pdb, `cl` asks "Clear all breaks?"; answer `y`.

**joint_command_node: [joint_command_node.cpp](joint_command/src/joint_command_node.cpp),
[joint_command_core.cpp](joint_command/src/joint_command_core.cpp)** (terminal 3)

| where | look at | why it matters |
|---|---|---|
| startup: `break JointCommandNode::JointCommandNode` before `run` | `n` through it | Config loading; `validateMitGains` refuses unsafe gains; MIT_ENTER goes out. Safe to stop: nothing is seeded yet. |
| `JointCommandNode::trySeedFromFeedback` | `p latest_feedback_` | **Read the position before the first move.** Live value: `{[10] = -105.5, [12] = 160.7, [13] = -92.9, [14] = -36, [22] = 41.5}`, motor-frame degrees. Stopping here stales the wrist (4.3). |
| `joint_command_core.cpp:121` | `p seeded`, `p unpowered_` | Converted to the command frame: `{0, 0, 0, 0, 0, -51.84}`. `unpowered_` = `{…, true, false}` (elbow.roll, motor 23, is silent). |
| `joint_command_core.cpp:541 if i == 5` (4.4) | `source_angles[i]`, `prev_targets_[i]`, `target`, `calibrated_deg` | Clamp → low-pass → velocity limit → delta limit → clamp → calibration, as in §3.1. |
| `JointCommandNode::publishMotorCommands` | `p cmds` | The MotorCmds as published. The wrist's has `control_type = 0` (MIT), `kp = 1.22`, `kd = 0.0098`, position in **rad**. |
| `JointCommandNode::mitFault` | `p reason` | Leave this one set as a normal `break` from the start. It costs nothing until it fires, and it tells you why the node gave up. |

**can_node: [can_node.cpp](can/src/can_node.cpp), [can_core.cpp](can/src/can_core.cpp)**
(terminal 2)

| where | look at | live values from the sim run |
|---|---|---|
| `can_node.cpp:251 if msg->motor_id == 22` (MIT, one-shot) | `p *msg`, `p p`, `p/x can_msg.data` | `position = 0.7406, kp = 1.22, kd = 0.0098` → `{0x87, 0x95, 0x80, 0x0, 0xa, 0x0, 0x88, 0x0}`. Decoding: pos code 0x8795 = 0.7404 rad; kp code 0x00A = 10 → 1.2207; kd 0x008 → 0.0098; v and t codes 0x800 → 0. |
| `can_node.cpp:191 if msg->motor_id == 10` (servo) | `p msg->position`, `p/x can_msg.id`, `p/x can_msg.data` | `-105.5` → id `0x8000040a` (0x80000000 = the extended-frame flag, 0x400 = position loop, 0x0A = motor 10) → `{0xff, 0xef, 0xe6, 0xe8}` = int32 −1 055 000 × 0.0001° |
| `can_node.cpp:408` | `p code` | MIT special frames: `FF FF FF FF FF FF FF FC` = enter, `FD` = exit |
| `can_core.cpp:103` **the last line before the bytes leave** | `p/x frame.can_id`, `x/8xb frame.data`, `bt` | `0x16` / `0xff … 0xfc`: enter-motor-mode to motor 22 |
| `can_node.cpp:472` (MIT feedback) | `p/x message.data`, `p fb` | `{0x16, 0x87, 0x6a, 0x80, 0x08, 0x00, 0x28, 0x00}` → status 1 (Enable), id nibble 6, 0.7242 rad = 41.50°, 40 °C |
| `can_node.cpp:553 if device_id == 10` (servo feedback) | `p/x message.data`, `p feedback_msg` | `0x290a`: `{0xfb, 0xe1, 0, 0, 0, 0, 0x28, 0}` → −105.5° (int16 × 0.1), 0 ERPM, 0 A, 40 °C, error 0 |

The backtrace at `can_core.cpp:103`, with the `std::function` plumbing removed (the settings in
`~/.gdbinit` do most of that trimming):

```
#0  CanCore::sendMessage             can_core.cpp:103     write() to the socket
#1  CanNode::publishCanMessage       can_node.cpp:566
#2  CanNode::sendMitSpecialFrame     can_node.cpp:408     (motor_id=22, code=0xFC "enter motor mode")
#3  CanNode::motorCMDCallback        can_node.cpp:259     switch on msg->control_type
#4-19  std::function / rclcpp::AnySubscriptionCallback::dispatch   ROS plumbing, skip
#20 rclcpp::Executor::execute_subscription
#22 rclcpp::executors::SingleThreadedExecutor::spin
#25 main                             can_node.cpp:575
```

and in joint_command, from the timer down to the seed:

```
#0  JointCommandNode::trySeedFromFeedback
#1  JointCommandNode::controlTimerCallback
#2-7   std::bind / rclcpp::GenericTimer::execute_callback   plumbing
#8  rclcpp::Executor::execute_any_executable
#9  rclcpp::executors::SingleThreadedExecutor::spin
#12 main
```

Both nodes are one executor thread running callbacks one at a time. That's why a slow callback
(or a breakpoint) delays everything else in that node, including its watchdog.

### 4.6 Shutting down cleanly

In gdb, Ctrl-C **stops** the node; it does not quit it. The node's Ctrl-C handling is a safety
feature (joint_command damps/exits its MIT joints on the way out), so deliver the signal
explicitly:

```
(gdb) signal SIGINT         <- the node runs its shutdown path and exits
(gdb) quit
```

Stop `joint_command` **before** `can_node`, so that its shutdown frames still have somewhere to go.

---

## 5. When to read the motor manual

gdb and the sim can prove the code does what the **code** intends. They cannot prove the
motor reads the bytes the same way. `gl40_sim.py` was written from the same manuals, so a sim run
that matches only shows the code and the sim agree. These are the moments where a manual table
is the real reference. For each one, find the table, work one example by hand, and compare it
with the bytes from §4.5.

| # | moment (where to stop) | what our code assumes | manual to check | risk if it's wrong |
|---|---|---|---|---|
| 1 | servo command, `can_node.cpp:191` | extended id = `(mode << 8) \| id`, mode 4 = position loop; payload int32 big-endian, 0.0001°/count (`dbc/humanoid.dbc` `PositionLoopCmd`) | AK series manual, servo-mode control section | wrong scale = wrong target angle |
| 2 | servo command, same stop | the drive moves to each new position at **its own configured speed/acceleration**. joint_command keeps each step small (≤ `velocity_max/50` per tick), but the motion *between* steps is the drive's. | AK manual: position-loop mode (4) vs position-velocity loop (6) and their speed/accel parameters; R-Link settings | one large step moves at the drive's full speed |
| 3 | MIT pack, `can_node.cpp:251` / `packMitCommand` | P/V/T/KP/KD ranges in `can/config/mit_profiles.yaml`. GL40: p ±12.5 rad, v ±200, t ±10, kp 0–500, kd 0–5. AK10-9/AK80-9: p ±12.56, v ±28/±65, t ±54/±18 | GL II manual V1.0 §5 (**and the ranges set in the CubeMars upper computer for that drive**); AK manual V3.2.0 §4.2 | **every MIT position/gain is scaled wrong**, so the drive is sent to a different position than intended. This is the screw-ripping failure mode. |
| 4 | MIT pack, same stop | field layout: pos 16 bits, vel/kp/kd/t 12 bits, byte order as in `mit_protocol.hpp`; `packMitValue` truncates, `packMitGain` rounds to nearest | GL II §5 worked example (pinned by `MitPacking.MatchesGlManualExample`) | wrong gain or position |
| 5 | MIT special frames, `can_node.cpp:408` | `FF×7 FC` enter, `FD` exit, `FE` set zero, `FB` clear errors, sent on the **standard** id = motor id. A GL II ignores MIT commands until it has received ENTER. | GL II §5; AK §4.2 | `FE` moves the zero: every calibration becomes wrong |
| 6 | MIT feedback, `can_node.cpp:472` | GL II reply on master id 0x000; byte 0 = status nibble \| id nibble; temps in bytes 6–7 | GL II §5 feedback table; master id as set in the upper computer | two drives with the same low id nibble can't be told apart (can_node refuses) |
| 7 | MIT feedback status, `mitStatusIsOk` / `checkMitFaults` | GL II: 0 = Disable, 1 = Enable, 8–E = faults. **AK: 0 = OK, 1 = over-temperature.** | both manuals' error/status tables | a fault read as healthy |
| 8 | AK MIT feedback, `decodeAkFeedback` | byte 0 = full id, byte 6 = temperature, byte 7 = error. `mit_protocol.hpp` says **"NOT YET CHECKED AGAINST A BENCH CAPTURE"** | AK §4.2 **plus a real `candump` from one AK in MIT mode** | position/fault misread. Required before any AK joint runs MIT. |
| 9 | servo feedback, `can_node.cpp:553` | id `0x2900 \| id`; pos int16 × 0.1°, speed × 10 ERPM, current × 0.01 A, temp int8, error uint8 | AK manual, servo-mode feedback message | seeding from a wrong angle, so the first move starts from the wrong place |
| 10 | drive timeouts | AK CAN timeout set in R-Link (100–200 ms recommended); GL II status 0xD = "communication loss" | AK / GL II parameter tables | the motor keeps its last command when the PC stalls |
| 11 | zero / origin | `SET_ORIGIN` temporary (0) vs permanent (1); AK80-9 joints lose their zero on power-up | AK manual, set-origin command | the arm moves toward a wrong zero. Recalibrate after every power-on (`ARM_BRINGUP.md`). |
| 12 | bus | 1 Mbps (`can/config/params.yaml`); 120 Ω termination at both ends | drive CAN settings | no communication (fails safe, but confusing) |

For rows 3, 8 and 9, a bench `candump` from the real drive is worth more than the manual: pin
it in `can/test/test_mit_protocol.cpp` the way `MitDecode.DecodesBenchFeedbackCapture` already
does for the GL40.

---

## 6. On the real arm: observe, never stop

With the motor supply on, run the normal Release nodes (`./watod up -d`, `ros2 launch
joint_command joint_command.launch.py`) with **no debugger attached**, and compare them against
the sim run:

```bash
candump -td -x can0                      # interfacing container: bytes on the wire
ros2 topic hz /interfacing/motorCMD      # ~250/s
tools/arm_roundtrip.sh --joints wrist.pitch --offset "0,0,0,0,0,5"   # host; plots the run folder
```

- The first MIT frame to motor 22 should decode (as in §4.5) to the **measured** position. The
  script also checks this and refuses if they differ by more than 2°.
- In the run folder's plot, `sp` (after limiting) must never be steeper than `velocity_max`
  and must never leave the joint limits. That check covers **every** tick, which a debugger
  can't do without slowing the node (rule 3).
- Keep the hardware E-stop within reach, supply cut-off in series with the 48 V line, for every
  run.

---

## 7. Troubleshooting

| symptom | cause / fix |
|---|---|
| `ls: cannot access '/root/ament_ws/src/...': Permission denied` | `sudo chmod o+x /root` (§2) |
| `No symbol "i" in current context` | you're in a different function or frame. `bt`, then `frame N`. |
| `<optimized out>` for a variable | you're running the `/opt/watonomous` Release binary. Use `~/dbg_ws/install/...`. |
| `nobody subscribes to /arm/joint_targets` | joint_command isn't up yet (gdb loads symbols first, which takes a few seconds), or it's on a different `ROS_DOMAIN_ID` |
| `MIT FAULT: ... feedback is X s old` right after the seed | a stop landed inside the seeding tick (4.3). `signal SIGINT`, restart the node, and raise the timeout in memory first. |
| `did not follow: wrist.pitch moved +0.4 of +10.0 deg` with no fault | joint_command was slowed or stopped: a conditional breakpoint on a per-tick line, or a long stop. `ros2 topic hz /interfacing/motorCMD` shows it. |
| wrist doesn't move at all, no fault | the drive never got MIT_ENTER. Check `can_node`'s log for `MIT enter motor mode -> motor 22`. |
| `ros2 node list` shows two `can_node`s | one is the container's own node on domain 0: your shell isn't on 42 |

When you're done: `signal SIGINT` / `quit` in both gdb sessions, Ctrl-C the sim. The debug
builds in `~/dbg_ws` and the installed gdb disappear when the containers are recreated; to
remove them sooner, `rm -rf ~/dbg_ws`.
