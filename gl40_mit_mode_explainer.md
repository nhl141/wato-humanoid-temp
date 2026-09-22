# Moving a CubeMars GL40 II in MIT mode — what happened, explained from scratch

This is a beginner-level walkthrough of a bench session on 2026-09-19. The request was simple:
*"I have a GL40 II in MIT mode on the CANable. Give me the command to move it 40 degrees; the
right KP/KD values should be in the repo."* It turned out that no such command existed, the
gains weren't in the repo, and the repo's own MIT code would have sent the motor somewhere
dangerous. This document explains every concept involved, what was found, what was built, and
what the motor actually did. If you already know CAN and PD control, skip to
[What was actually done, step by step](#what-was-actually-done-step-by-step).

---

## 1. The pieces of hardware and software involved

### The motor: CubeMars GL40 II
A small "gimbal" motor (the kind used for camera stabilisers) with a built-in driver board.
Key numbers: **0.25 N·m rated torque, 0.73 N·m peak**. That is tiny — about the torque needed to
turn a stiff door knob. In this robot it's the wrist / gripper motor. The bigger arm joints use
CubeMars AK10-9 and AK80-9 actuators, which are a different product family with a *different*
driver firmware — that difference matters later.

### The bus: CAN
CAN is a two-wire network (CAN_H / CAN_L) used in cars and robots. Devices send **frames**: a
short message with an **ID** (11 bits = 0–2047, or 29 bits for "extended" frames) and up to
**8 bytes of data**. There is no "address" beyond the ID — every device sees every frame and
decides whether it cares based on the ID. Our bus runs at **1 Mbps**. Both ends of the wire need a
120 Ω terminating resistor or the signals reflect and nothing works.

### The adapter: CANable, slcan, SocketCAN
The **CANable** is a USB stick that puts CAN frames on the wire. Linux sees it as a serial port
(`/dev/ttyACM0`, symlinked to `/dev/canable` by a udev rule in this repo). A daemon called
**slcand** ("serial-line CAN daemon") turns that serial port into a proper Linux network
interface called **`can0`**. Once `can0` exists, any program can send and receive CAN frames
through the standard Linux **SocketCAN** API — the same `socket()` calls you'd use for TCP, just
with `AF_CAN`. The command-line tools `candump` (print every frame on the bus) and `cansend`
(send one frame) come from the `can-utils` package, which is installed inside the
`interfacing` Docker container but not on the host.

### The repo's pipeline (what normally talks to the motors)
```
ROS topic /arm/joint_targets  (ArmPose, degrees)
        │  joint_command_node  (C++: clamps, velocity limit, calibration offsets)
        ▼
ROS topic /interfacing/motorCMD  (MotorCmd: motor_id, control_type, position, kp, kd, …)
        │  can_node  (C++: packs the MotorCmd into a CAN frame using a DBC file)
        ▼
can0  ──►  CANable  ──►  wire  ──►  motor
```
A **DBC file** (`src/interfacing/dbc/humanoid.dbc`) is a text format from the automotive world
describing "message X has ID Y; bits 7..0 mean this, bits 15..8 mean that". `can_node` reads it
and uses a library to place numbers into the right bits. `config/mit_profiles.yaml` tells
`can_node` the number ranges for each motor (more on that below).

---

## 2. What "MIT mode" means

CubeMars drivers have two families of control:

* **Servo mode** — you send "go to 90 degrees" or "spin at 500 rpm" and the driver's internal
  controller does the rest. The repo's `POSITION_LOOP` etc. control types are these. They use
  *extended* 29-bit CAN IDs like `0x400 | motor_id`.
* **MIT mode** (named after the MIT Cheetah robot protocol) — you send *five numbers* every
  few milliseconds and the driver computes the motor torque yourself-style:

  ```
  torque = kp · (p_des − p_actual)  +  kd · (v_des − v_actual)  +  t_ff
  ```

  | name | meaning | unit |
  |---|---|---|
  | `p_des` | where you want the shaft to be | radians |
  | `v_des` | how fast you want it moving | rad/s (the GL II manual says "r/s") |
  | `kp` | **stiffness** — how hard it pulls per radian of error | N·m per rad |
  | `kd` | **damping** — how hard it resists velocity error | N·m per rad/s |
  | `t_ff` | extra constant torque ("feed-forward") | N·m |

  This is a **PD controller** (proportional + derivative). Think of `kp` as a spring pulling the
  shaft toward `p_des` and `kd` as a dashpot (shock absorber). If `kp = 0` and `kd = 0` the motor
  produces zero torque and you can turn it freely by hand. With a big `kp` it becomes rigid.

  **The crucial safety consequence:** torque = kp × error. If the shaft is 40° (0.7 rad) away
  from `p_des` and kp is 100, the driver asks for 70 N·m — far more than this motor can make, so
  it slams to its current limit at full speed. Low kp + small error = gentle. That is why the
  script ramps the setpoint slowly instead of jumping.

### Why kd must not be zero
The GL II manual warns: "When controlling position, kd cannot be assigned 0, otherwise it will
cause the motor to oscillate and even go out of control." A spring with no damping bounces
forever. So even a tiny kd is mandatory.

### How five numbers fit in 8 bytes
CAN frames carry 8 bytes = 64 bits. The five values are squeezed in as fixed-point integers:
position gets 16 bits, the other four get 12 bits each (16 + 4×12 = 64). To convert a real
number into its integer code you need to know the **range** the driver expects:

```
code = (x − x_min) · 2^bits / (x_max − x_min)        # "float_to_uint" in the manual
```

Example: position 2 rad with range ±12.5 → `(2 + 12.5) · 65536 / 25 = 38010 = 0x947A`.

The ranges are **settings inside the driver** (visible in CubeMars' Windows "upper computer"
tool). GL II defaults: position ±12.5 rad, velocity ±200, torque ±10 N·m, kp 0–500, kd 0–5.
If your code assumes ±12.5 but the driver is set to ±50, every position you send is wrong by
4×. This is why the session included a "monitor" step to check the scale before moving.

Because kp has 12 bits across 0–500, its resolution is `500 / 4096 ≈ 0.122` — you cannot ask for
kp = 0.3, you get 0.244 or 0.366. Same for kd: steps of `5 / 4096 ≈ 0.0012`.

Byte layout for a command (from the manual, and the same in every CubeMars MIT product):
```
byte 0: position bits 15..8         byte 4: kp bits 7..0
byte 1: position bits 7..0          byte 5: kd bits 11..4
byte 2: velocity bits 11..4         byte 6: kd bits 3..0 | t_ff bits 11..8
byte 3: velocity bits 3..0 | kp 11..8   byte 7: t_ff bits 7..0
```

### Special frames
Four magic 8-byte messages, all `FF FF FF FF FF FF FF xx`:
`FC` = enter motor mode (enable), `FD` = exit motor mode (motor goes limp), `FE` = set the
current position as zero, `FB` = clear errors.

### Feedback
After every frame the driver replies with 8 bytes on the **master ID** (default `0x000`):
```
byte 0: error code (high nibble) | motor id low nibble
byte 1-2: position (16 bit)      byte 3 + high nibble of 4: velocity (12 bit)
low nibble of 4 + byte 5: torque (12 bit)
byte 6: driver temperature °C    byte 7: motor temperature °C
```
Error codes: 0 = disabled, 1 = enabled, 8 = over-voltage, 9 = under-voltage, A = over-current,
B = MOSFET over-temp, C = winding over-temp, D = communication loss, E = overload.

### CAN IDs in MIT mode
The GL II uses **standard 11-bit** IDs: `ID = (mode << 8) | node_id`, and MIT mode is mode 0.
So for a motor configured as node 22 the command ID is simply `22 = 0x016`.

---

## 3. Why the repo couldn't do it

### There are no GL40 gains anywhere
`joint_command/config/safety_limits.yaml` sets `mit_kp: 0.0` and `mit_kd: 0.0` for every joint
with the comment "safe no-op, start LOW". `can/config/mit_profiles.yaml` lists ranges for the five
AK-series arm motors (ids 10–14) only. The only "GL40 gains" in the repo are for the Isaac Sim
simulation (`stiffness=341, damping=18`), which are in the simulator's units and would be wildly
wrong on hardware. So the premise "the values should be in the repo" was false.

### The MIT path in `can_node` has never worked on a GL II drive
Four independent problems, any one of which is fatal:

1. **No profile for id 22.** `can_node.cpp` refuses to send an MIT frame for a motor with no entry
   in `mit_profiles.yaml` (deliberately — better than sending unscaled garbage).
2. **Wrong byte order in the DBC.** `humanoid.dbc` lays the fields out as `kp, kd, position,
   velocity, torque`. The real protocol is `position, velocity, kp, kd, torque`. If you sent
   "40°" through it, the position bits would land in the driver's kp field (kp ≈ 222 — a
   battering ram) and the kp bits would land in the position field (p_des ≈ −12.5 rad, the far
   end of the range). The motor would have slammed toward −12.5 rad as hard as it could.
3. **Extended instead of standard IDs.** Every frame `can_node` sends carries the 29-bit
   "extended" flag (that's what the AK servo protocol wants). A GL II in MIT mode listens only
   to 11-bit standard frames, so it would ignore the command entirely — which, given bug 2, is
   lucky.
4. **No enter/exit-motor-mode frames and no MIT feedback decoding.** `can_node` only understands
   the servo-mode feedback layout (`0x2900 | id`); MIT replies on ID `0x000` would be dropped.

Conclusion: the shortest safe path was a small standalone script that speaks the protocol
directly over SocketCAN, and to leave fixing `can_node` as a follow-up.

---

## 4. What was actually done, step by step

### Step 0 — safety checklist
This repo has a mandatory `real-hardware-safety` skill (`.claude/skills/real-hardware-safety/`)
written after an incident where sim-vs-real zero mismatch made the arm jerk and strip motor
mount screws. Its rules shaped everything below: read the current position before moving, move
relative to it, ramp slowly, clamp torque, have a software E-stop, and always remind about the
hardware E-stop.

### Step 1 — read the repo, then the manual
Searching the repo found the four bugs above. The CubeMars **"Gimbal Motor Drive User Manual
V1.0 — For GL II"** PDF was downloaded and its section 5 (CAN protocol) extracted; every
constant in the script cites it.

### Step 2 — check the bus without sending anything
```
ls -l /dev/canable            → symlink to ttyACM0  (adapter present)
ip -details link show can0    → UP, state ERROR-ACTIVE (bus is electrically healthy)
pgrep slcand                  → slcand -o -c -s8 /dev/canable can0  (1 Mbps)
candump can0  (4 s)           → nothing; RX 0 packets, TX 0 packets since boot
```
"Silent" was expected: a GL II only speaks when spoken to, and nothing had ever transmitted on
this interface. `ros2 node list` showed only `/can_node`; `joint_command_node` was not running
(good — it would spam position commands to id 22 at 50 Hz).

### Step 3 — the probe (first transmission, with your OK)
Three copies of one specially chosen frame were sent, to ids 21, 22 and 1:
```
7F FF 7F F0 00 00 07 FF
```
Decoded: position = mid-range (0 rad), velocity = mid-range (0), **kp = 0, kd = 0**, t_ff =
mid-range (0 N·m). With both gains zero the torque formula gives 0 whatever the ranges are, so
this frame cannot move the motor. Only id 22 replied, on ID `000`:

```
16 99 21 7F E7 FF 28 00
│  │──┘ │─┘└─┘ │  └─ motor temperature 0 °C (no sensor wired)
│  │    │   │  └──── driver temperature 0x28 = 40 °C
│  │    │   └─────── torque code 0x7FF = 2047 → ≈ 0 N·m
│  │    └─────────── velocity code 0x7FE = 2046 → ≈ 0
│  └──────────────── position code 0x9921 = 39201 → 39201·25/65535 − 12.5 = +2.454 rad (≈140.6°)
└─────────────────── 0x1 = "Enable", 0x6 = low nibble of id 22 (0x16)
```
So: one motor, id 22, already enabled, sitting at about 2.45 rad on its own scale, healthy.

### Step 4 — write the script `gl40_mit_move.py`
About 550 lines, Python standard library only (no ROS, no `python-can`). It lives in
`src/interfacing/can/scripts/` next to `calibrate_arm.py`. What it does, in order:

1. **Refuse bad gains up front.** kd must be > 0. Gains are snapped to the nearest 12-bit code
   and printed as the driver will see them. Then it checks
   `kp × max_track_err ≤ max_torque` — explained in step 9.
2. **Print the hardware E-stop reminder.**
3. **Install Ctrl-C / SIGTERM handlers** and a `finally:` block so that *whatever* happens —
   abort, crash, Ctrl-C — the script sends "exit motor mode" three times and the motor goes limp.
4. **Wake:** send enter-motor-mode, wait ≤ 0.5 s for feedback, decode it, abort if the motor is
   silent or reports an error. This is the "read where you are before moving" rule.
5. **`--monitor` mode** (optional): loop at 10 Hz sending the zero-gain frame and printing the
   decoded position, so you can turn the shaft by hand and confirm the scale (a quarter turn
   should read ≈ 1.571 rad). This is how you verify the ±12.5 rad assumption.
6. **Compute the target** = current + 40° (relative by default; `--absolute` opts out) and refuse
   if it's outside ±p_max. Stretch the ramp time so the setpoint never moves faster than
   `--max-setpoint-vel` (1 rad/s).
7. **Hold** the current position for 1 s with the chosen gains. If the gains are wrong, this is
   where you find out, with zero error and therefore zero torque.
8. **Ramp** the setpoint linearly to the target over 4 s at 50 Hz. Every tick: send frame, read
   feedback, and abort if torque > 0.3 N·m, tracking error > limit, temperature > 60 °C, driver
   error code, or 200 ms without a reply.
9. **Settle** 2 s at the target, print the final error, then free the motor — or with `--hold`,
   keep holding until Ctrl-C.

It also has `--selftest` (packs the manual's worked example `pos 2 rad, kp 0.123, kd 0.005` and
checks it produces the manual's bytes `94 7A 7F F0 01 00 47 FF`, and decodes the probe reply
above) and `--dry-run` (prints the frames it *would* send). Both passed on the host and inside
the container before any real move.

### Step 5 — first monitor run (8 s, you didn't touch it)
The reported position wandered about 12° and single samples jumped up to 5°. Two possible
explanations: a noisy encoder (bad — you can't close a loop on noise) or something physically
moving the shaft. Not safe to guess, so:

### Step 6 — second monitor run (15 s, you turned it ~10–20°)
For the first two seconds, untouched, the reading was `2.4748 – 2.4756 rad` — a jitter of
±0.02°. The encoder is fine. The total swing was 14.5°, matching your estimate, which confirms
the ±12.5 rad scale. (The earlier "drift" is explained in step 8.)

### Step 7 — first real move: `--deg 40` with kp 0.366 → **aborted safely**
```
Move: +2.5214 → +3.2195 rad (delta +40°) over 4 s; worst-case PD torque 0.256 N·m
[ramp] sp=+2.70  pos=+2.63  τ=+0.027
[ramp] sp=+2.88  pos=+2.74  τ=+0.051
[ramp] sp=+3.07  pos=+2.86  τ=+0.076
ABORT: tracking error 15.2 deg exceeds 15.0 deg
Freeing motor (exit motor mode)...
```
The motor moved (+22°) but fell further and further behind the setpoint until the 15° abort
fired. Why: kp 0.366 × 15° (0.26 rad) = only 0.095 N·m, and this shaft evidently needs more than
that to keep moving. The safety net did exactly what it should — peak torque 0.076 N·m, no
drama.

### Step 8 — two 20° moves with `--kp 0.61` → completed, and revealed the load
Both steps ran without aborting, but:

* Each step ended with a steady **6° error** while holding **+0.05 N·m**. A PD controller only
  produces torque when there is error; a constant torque demand therefore produces a constant
  sag. Something is pushing back on the shaft.
* When step 1 freed the motor, step 2 woke up at **142°** — *lower than where step 1 started*.
  The shaft **falls back** to a rest position when unpowered. That's a gravity- or spring-like
  load, and it also explains the "drift" in the first monitor run.
* The printed gain was `kp=0.4883 (raw 4)`, not 0.61. Cause: `0.61 × 4096 / 500 = 4.997`, and the
  manual's packing function *truncates*, so 4.997 became 4. A rounding trap.

### Step 9 — two script fixes
1. **Gain snapping.** The script now rounds to the nearest code, prints `(raw N)`, and nudges the
   value it packs up by half a count so truncation lands on exactly that code.
2. **A better worst-case rule.** The original rule refused any `kp × |40°| > 0.3 N·m`, i.e. "what
   if the ramp were skipped". But the ramp can't be skipped, and if the motor stalls the
   tracking-error abort fires at `max_track_err`. So the true worst case is
   `kp × max_track_err`. With `--max-track-err 12` this allows kp up to ~1.4 while keeping the
   stalled torque under 0.3 N·m.

Both verified with `--selftest` and `--dry-run` (`--kp 0.61` now shows `…005…` in the frame,
`--kp 1.22` shows `…00A…`; 1.22 with a 15° limit is refused, with 12° accepted).

### Step 10 — final move: `--deg 40 --kp 1.22 --max-track-err 12 --hold`
```
gains as the drive will see them: kp=1.2207 N.m/rad (raw 10)  kd=0.00977 (raw 8)
  pos=+2.5053 rad (+143.5°)  err=Enable  τ=−0.002  drive=41C
Move: +2.5053 → +3.2035 rad (delta +40°) over 4 s; worst-case (stalled) PD torque 0.256 N·m
[ramp]   sp=+148.7°  pos=+147.8°  τ=+0.017
[ramp]   sp=+159.1°  pos=+156.4°  τ=+0.056
[ramp]   sp=+169.5°  pos=+165.2°  τ=+0.085
[ramp]   sp=+179.9°  pos=+174.4°  τ=+0.115
[settle] sp=+183.5°  pos=+177.8°  τ=+0.125
Reached: pos=+3.1023 rad (+177.8°), error −5.79°
Holding at target. Ctrl-C to free the motor.   … 15 s, position steady to ±0.05° …
[signal 2] stopping — motor will be freed
Freeing motor (exit motor mode)...
```
Result: **+34.2° of the requested 40°**, held rock-steady at **0.125 N·m** (half the rated
torque, well under the 0.3 N·m ceiling), 41 °C, no errors, released cleanly on Ctrl-C.

The 5.8° shortfall is arithmetic, not a bug: `0.125 N·m / 1.22 N·m/rad = 0.10 rad = 5.9°`. The
load torque grows with angle (0.05 N·m at 157°, 0.125 at 178°), which is what gravity on an
attached lever does. Pure PD under a 0.3 N·m safety ceiling cannot do better against that load;
closing the last few degrees needs either torque feed-forward (`t_ff` = an estimate of the
gravity torque) or the driver's position-velocity mode, which has an integrator.

---

## 5. How to run it yourself

Everything happens inside the `interfacing` container, because that's where `can0` and
`can-utils` live. The repo is mounted at `/root/ament_ws/src/interfacing`, a directory only root
can read, hence `sudo`.

```bash
./watod up -d                 # ACTIVE_MODULES must include "interfacing"; can_node brings up can0
./watod -t interfacing        # shell into the container
S=/root/ament_ws/src/interfacing/can/scripts/gl40_mit_move.py

sudo python3 $S --selftest                       # 1. no bus: packing matches the manual
sudo python3 $S --id 22 --monitor                # 2. zero torque; turn shaft, watch rad, Ctrl-C
sudo python3 $S --id 22 --deg 40 --dry-run       # 3. print the frames only
sudo python3 $S --id 22 --deg 40 --kp 1.22 --max-track-err 12 --hold   # 4. the move; Ctrl-C frees it
```

Before step 4, every time:
* a **hardware E-stop** on the motor supply within reach (software exit-motor-mode is a backup,
  not a substitute — if the PC freezes only the hardware switch saves you);
* `joint_command_node` **not** running;
* the shaft can turn +40° from where it currently sits without hitting anything;
* if you changed anything in the CubeMars upper computer, re-check `--p-max/--v-max/--t-max`
  match the driver's parameter page.

Useful flags: `--deg -40` (other direction), `--absolute` (target on the driver's own zero),
`--duration 8` (slower), `--rad`, `--id`, `--master-id`, `--max-torque` (don't raise past the
motor's 0.73 N·m peak), `--max-temp`.

Reading the live line: `sp` = setpoint the script is sending, `pos` = where the driver says the
shaft is, `tau` = torque the driver reports, `err=Enable` = driver status word.

---

## 6. The numbers, and where they came from

| quantity | value | why |
|---|---|---|
| kp (default) | 0.366 N·m/rad (raw 3) | so a full 40° error stays < 0.3 N·m; manual's example is 0.123 |
| kp (used for the loaded shaft) | 1.22 N·m/rad (raw 10) | with a 12° abort limit the stalled torque is 0.26 N·m |
| kd | 0.0098 N·m·s/rad (raw 8) | must be non-zero; manual's example is 0.005 |
| v_des, t_ff | 0 | pure position PD; speed comes from ramping the setpoint |
| ramp | 40° in 4 s = 0.17 rad/s | ≪ the skill's 3 rad/s GL40 test limit |
| torque abort | 0.3 N·m | skill's GL40 test ceiling (rated 0.25, peak 0.73) |
| temperature abort | 60 °C | skill recommendation |
| feedback timeout | 200 ms | skill's 100–200 ms CAN watchdog range |

None of these came from the repo; they come from the safety skill's limits plus the GL II manual.

---

## 7. What's left

Everything in the original list has since been done (see
`src/interfacing/TESTING_LIMITS_AND_TELEMETRY.md` for how to exercise it):

* `humanoid.dbc`'s MIT byte order is fixed and `MITControlCmd` is now a standard 11-bit
  message, so `can_node` puts a correct frame on the wire.
* `mit_profiles.yaml` has GL40 entries for ids 22 and 21 (`family: gl2`), MIT feedback on the
  master id is decoded into `MotorFeedback` (now carrying `torque`), and `MotorCmd` has
  `MIT_ENTER / MIT_EXIT / MIT_SET_ZERO / MIT_CLEAR_ERRORS`. `ros2 topic pub
  /interfacing/motorCMD` works for these motors like it does for the AK ones.
* The gains live in `joint_command/config/safety_limits.yaml` (`mit_kp: 1.22`, `mit_kd: 0.0098`
  for the wrist), and `joint_command` refuses to start if they break the stall-torque rule. It
  also runs a torque / tracking / feedback watchdog and frees the drives on any fault.
* Gain quantisation is handled in `can_node` (nearest code), so the "0.61 became 0.488"
  truncation trap from step 8 cannot recur.

Still open:

* `hardware_mapping.yaml`'s wrist/gripper entries (`zero_offset: 0`, ±90° limits) are still
  placeholders. Because the real wrist sits near 143°, `joint_command` **excludes** it from
  ROS-driven motion until `calibrate_arm.py` has been run for it — loudly, rather than clamping
  it and walking it to the limit.
* Gravity feed-forward, if the wrist ever needs to hold an exact angle under load. The ~5°
  sag measured here is what pure PD under a 0.3 N·m ceiling gives you.
* MIT feedback decoding for the AK family (they run POSITION_LOOP and report through
  `ServoStatusFeedback`, so nothing needs it today).

---

## Glossary

* **CAN frame** — one message on the bus: an ID plus up to 8 data bytes.
* **Standard / extended ID** — 11-bit vs 29-bit CAN identifier. Same wire, different header.
* **SocketCAN** — Linux's built-in CAN networking; `can0` is an interface like `eth0`.
* **slcand / CANable** — daemon + USB adapter that create `can0` from a serial port.
* **DBC** — text file describing which bits of which CAN message mean what.
* **MIT mode** — CubeMars' low-level control mode: you send p, v, kp, kd, t_ff at high rate.
* **PD controller** — torque = kp·(position error) + kd·(velocity error).
* **kp / kd** — stiffness / damping gains. Low = soft and safe; high = rigid and violent.
* **Setpoint** — the position you are asking for right now (`sp` in the output).
* **Ramp** — moving the setpoint gradually so the error, and hence torque, stays small.
* **Feed-forward (t_ff)** — torque added regardless of error, e.g. to cancel gravity.
* **Fixed-point packing** — converting a real number to an integer code over a known range.
* **Master ID** — the CAN ID the driver uses for its replies (default 0).
* **Enter / exit motor mode** — the `…FC` / `…FD` frames that enable / free the motor.
