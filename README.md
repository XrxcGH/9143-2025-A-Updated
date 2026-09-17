# Team 9143 - 2025 Robot Code (2026 Libraries)

This repository contains the code for Team 9143's 2025 FRC robot, updated to the 2026 WPILib/vendor libraries. The drivetrain and CorAl run Kraken X60s through CTRE Phoenix 6; the elevator runs NEOs on Spark MAX controllers through REVLib.

---

## Robot Overview

- **Swerve Drivetrain**: CTRE Phoenix 6 swerve on a 30"×30" frame — mixed modules (front: SDS MK4i, back: SDS MK4n), **all with L3+ drive gearing**, all Kraken X60 drive/steer, CANcoders, and a Pigeon 2. Drive/coupling ratios are uniform; only the steer ratios differ and are set per module.
- **Elevator Subsystem**: Dual NEO/Spark MAX elevator (15:1 MAXPlanetary) with MAXMotion height control in inches.
- **CorAl (Coral and Algae) Subsystem**: Pivoting arm + intake rollers with absolute-encoder-referenced angle control and CANrange game piece detection.
- **Vision System**: Three Limelights fusing MegaTag2 pose estimates into odometry, plus AprilTag tracking.
- **LED System** *(code commented out — there is no CANdle on the robot)*: CTRE CANdle robot-state patterns, kept in [LEDs.java](src/main/java/frc/robot/subsystems/LEDs.java) for when one is installed.
- **Autonomous**: PathPlanner routines **and** Choreo trajectories, both selected from one dashboard chooser.

---

## Controls

### Driver (Xbox controller, port 0)
| Input | Action |
|---|---|
| Left stick | Field-centric translation (scaled by the *Teleop Speed Scale* tunable — 25% by default for indoor testing) |
| Right stick X | Rotate |
| A (hold) | X-lock wheels (brake) |
| B (hold) | Point modules at left-stick direction |
| Y (press) | Toggle AprilTag vision tracking (goal follows the operator's selected pose) |
| Left/Right trigger | Align on the LEFT / RIGHT reef branch (L2–L4 tracking) |
| D-pad | Slow robot-centric nudges |
| Left bumper | Re-zero field-centric heading |
| Back/Start + X/Y | SysId characterization (testing only) |

### Operator (Xbox controller, port 1)
All position buttons run coordinated elevator + arm sequences through the **Superstructure**, which automatically avoids mechanism contact from any starting pose.

| Input | Action |
|---|---|
| Left stick Y | Elevator manual control (holds height on release; **no interlocks**) |
| Right stick X | CorAl pivot manual control (holds angle on release; **no interlocks**) |
| D-pad down | Coral **L1** pose (0", 100°) |
| D-pad left | Coral **L2** pose (12", 10° — 5° geometric plus chain-backlash sag) |
| D-pad right | Coral **L3** pose (29", 22.5°, rotated 1" below the target) |
| D-pad up | Coral **L4** pose (52.5", 20°, rotated at the 33" station) |
| A | Coral intake (stow to base, rollers until CANrange confirms) |
| X | Eject coral at the current pose (rollers 0.5 s) |
| B | Algae **low** intake (20.5", 160°, rollers in) |
| Back | Algae **high** intake (37.5", 160°, rollers in) |
| Y | Algae hold (hold rollers, arm to 100° — the safe travel angle) |
| Right bumper | Algae score (52.5", 105°, then eject) |
| Left trigger | Stow to base (rollers stopped, arm tucked) |
| Right trigger | Raise arm to safe travel angle (context-aware) |
| Left bumper | Reset elevator encoders (**disabled only**, mechanism at base) |
| Start | Reset CorAl pivot encoder (**disabled only**, mechanism at base) |

---

## Subsystems

### Swerve ([Swerve.java](src/main/java/frc/robot/subsystems/Swerve.java))
Extends the Phoenix 6 `SwerveDrivetrain` (high-frequency odometry and module control run in CTRE's code). Adds PathPlanner `AutoBuilder` configuration, vision pose fusion (with the required FPGA-to-Phoenix timestamp conversion), an AprilTag tracking command, SysId routines, and alliance-aware operator perspective.

**Mixed modules**: the front pair are SDS MK4i and the back pair SDS MK4n, both with **L3+ drive gearing** (5.36:1, 16T pinion) — so drive and coupling ratios are identical on all four modules, and only the **steer ratios differ** (MK4i 150/7:1 front, MK4n 18.75:1 back), set per module in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java). All driving (teleop and path following) uses **closed-loop velocity** so wheel speeds track the request regardless of battery sag. Top speed ~5.96 m/s at 12 V, matched by PathPlanner's `maxDriveSpeed`.

### Elevator ([Elevator.java](src/main/java/frc/robot/subsystems/Elevator.java))
Two NEOs on Spark MAX controllers, each through a 15:1 MAXPlanetary reduction (5:1 × 3:1 cartridges — the second 3:1 was removed in Sept 2026 for speed), a 1:1 90° gearbox, and a ½" hex shaft driving 22T #25 sprockets — 5.5" of chain per sprocket rotation, doubled by the cascade rigging. The gearing predicts **0.733" of carriage travel per motor rotation** (0.244" at the earlier 45:1, which 20" and 40" tape tests confirmed to within 0.5%; the *Elevator – Travel Ratio* tunable stays at 1.0 and exists only for a re-check). Heights are measured from the **top of the base-stage 2×1 to the bottom of the carriage 2×1**, and in that frame the carriage sits at **≈0.875"** when it rests on its hard stop (*Elevator – Height At Hard Stop*), so the encoder is referenced *to* that value rather than zeroed — a commanded 20" lands the tape at 20". Travel is 0.875–53" — see *Calibrating the elevator height* below. The right controller is a hardware follower of the left, and encoder conversion factors scale everything to **inches**. The follower carries **no soft limits of its own**: REV documents only that a follower mirrors the leader's voltage output, and since the follower spins opposite the leader its encoder counts negative as the carriage rises — leader limits copied onto it would keep its reverse limit at zero in force for the whole climb and, if enforced, turn it into a brake the leader has to drag (slow, stuttering, current-limited climbs). Height moves use MAXMotion profiles on the controller with on-controller kS/kV/kG feedforward (REVLib 2026), so the carriage tracks smoothly and holds height at targets and when the operator releases the stick. Soft limits bound travel in every control mode, voltage compensation keeps response consistent as the battery sags, and configuration is persisted to flash so a brownout can't revert it.

**Elevator gearing (15:1, Sept 2026):** the NEO free speed through 15:1 is ≈69 in/s at the carriage, so the shipped 50 in/s cruise is ≈72% of it (≈8.7 V of kV at cruise, leaving room for the position loop on a sagging battery) and full travel takes ≈1.2 s. Holding costs ≈6 A per motor before the constant-force springs help, so kG starts at 0.6 V and should be tuned down until the carriage just holds. One real cost of the lighter gearing: with the robot disabled at height, brake mode lets the carriage creep down ≈3 in/s — stow before disabling.

### CorAl ([CorAl.java](src/main/java/frc/robot/subsystems/CorAl.java))
Pivot arm + intake rollers (both Kraken X60). The REV Through Bore absolute encoder is the angle reference: the motor's sensor is seeded from it at startup, before each move, and whenever the arm is idle — never mid-move. Motion Magic in degrees, closed-loop holding at targets. Game piece detection uses the CANrange's on-device proximity bit — *distance below the threshold*, with a hysteresis band on both sides, only while the return is strong enough to be a valid measurement — rejected while the sensor reports a compromised measurement, then debounced 0.3 s on **both** edges; a confirmed arrival stops the intake automatically.

**If the sensor "sees" a coral that isn't there:** the empty claw's own structure is sitting near the threshold, so the bit chatters (the original 0.1 m threshold with the 1 cm default hysteresis did exactly this). On the Testing tab read `CorAl/CANrange Distance` with the claw empty and with a coral held, and set *CorAl – Coral Detect Distance* halfway between (it applies the next time the robot is disabled; *Hysteresis* widens the dead band). `CANrange Raw Detect` shows the sensor's verdict before the debounce, `Signal Strength` must be above 2500 for anything to count, and `Health` should read Good. The field of view is narrowed to 13.5° so plates and rollers at oblique angles stay out of the beam.

### Superstructure ([Superstructure.java](src/main/java/frc/robot/Superstructure.java))
The command factory for all coordinated elevator + CorAl motion. The free region of (elevator height, arm angle) was computed from the robot CAD — every CorAl part's true outline swept about the pivot against the base stage, the middle stage (which rises at *half* the carriage travel), the funnel, frame and bumpers — and validated against the two contacts measured on the robot (0° at ≈10.75", 5° at ≈23", both reproduced within ½"). See *CAD clearance analysis* below for the map. The planner moves through that region in **staged, state-gated steps**: every trigger is a measured height or angle, never a timer, so the sequences stay safe at any elevator or pivot speed — only their duration changes.

- **Low box.** Any arm angle from 8° up is clear below 16.5", so base→L2 and base→L1 move both mechanisms together. Tucked (< 8°) is allowed only below 8.5" (the cross bar under the top sprockets).
- **Climbing out** needs the arm at ≥ 75° — the middle-stage top tube blocks roughly 18–30" for lower angles — and from **RAISE = 100°** the carriage may go anywhere. **90° is not a travel angle**: the claw's lower rear meets the middle-stage top tube from ≈36" up. Ascents give the elevator a ratcheting head start (8.5" while tucked → 16.5" once clear → 36" past 75° → anything once the arm reads RAISE, ≥ 97°).
- **L3 (29", 22.5°)**: climb at RAISE, rotate 1" below the target. Leaving, the carriage **lifts to 31"** while the arm swings up and only descends once the arm has cleared 75° (band A) — the fix for the return that hit the tube.
- **L4 (52.5", 20°)**: the top of travel is clear only at ≤ 22.5°, and the 100°→20° rotation is impossible up there, so the carriage stops at the **33" station** (where 25–100° are all clear), the arm rotates to 25°, the carriage climbs to 48", the arm finishes to 20° above 37", and only then does the carriage go to the top. Leaving mirrors it: **drop first** to 39" at 20°, stage to 45° below 43", drop to the station, swing to RAISE below 37", then descend.
- **Algae poses (160°)** hit the bumper below ≈7.5", so the arm waits at 110° until the carriage is above 8" — and, leaving an algae pose for anything low, the arm comes back to RAISE before the carriage drops below 8".
- **Stow** tucks to 0° once the descending carriage passes 8.5".

Simulated against the full CAD model with the 15:1 elevator (50 in/s, 400 in/s²) and the softened pivot profile, sampled every 10 ms, every transit clears by ≥ 1.0" (model figure; about 1–1.5" real), including with the arm slowed to 150°/s by chain backlash. `SuperstructureCorridorTest` pins the station heights and the presets to the corridor table, so an edit that breaks the geometry fails the build rather than the robot.

All overlaps are gated on measured heights/angles, never timing, and every command first safely escapes the **current** pose — buttons are safe in any order at any time. Manual stick control bypasses these interlocks.

### Vision ([Vision.java](src/main/java/frc/robot/subsystems/Vision.java))
Sends the gyro heading to each Limelight and fuses the returned **MegaTag2** pose estimates into the drivetrain with distance/tag-count-scaled confidence. Caches the closest **trackable** target once per loop for the tracking command and dashboard. Requires Limelight OS 2026.0+.

**Tracking goals** are resolved per tag class and superstructure state (2025 Reefscape tags):
| Tags | Goal |
|---|---|
| Reef (6–11, 17–22), L2–L4 selected | Bumpers **flush** with the reef base, centered on the driver-selected **left/right branch** (±`REEF_BRANCH_OFFSET`) |
| Reef, L1 selected | Centered, held **1 m** away so the arm can swing to 100° without hitting |
| Reef, algae/other | Flush and centered |
| Coral stations (1, 2, 12, 13) | **Rear** bumpers flush with the wall, centered — approached backward via the rear-facing funnel camera (drive terms mirrored automatically) |
| Barge (4, 5, 14, 15) + processor (3, 16) | Intentionally blank — the tracker ignores them |

All "flush" distances are what the camera *reads* in that condition — tune by physically placing the robot in the goal position and copying the `Vision/Distance` dashboard value into the constant.

**Camera mounting (MegaTag camera poses).** Each Limelight needs its lens position and orientation on the robot so MegaTag can turn "where the tag is in the image" into "where the robot is on the field". These live in `VisionConstants.LIMELIGHT_POSES` and are pushed to the cameras at startup — so they're version-controlled and survive a camera reset — but only for cameras marked *measured*. Limelight's robot-space convention: origin at the frame center on the floor, **X forward, Y toward the robot's right** (opposite of WPILib), Z up; pitch positive = lens tilted up; yaw = lens heading (180° = rear-facing).

| Camera | Forward | Side | Up | Pitch | Yaw | Status |
|---|---|---|---|---|---|---|
| funnel (rear) | −14.0" (1.0" inside the back edge of the 30" frame) | 0 (centered) | 29.625" | +50° (tilted up) | 180° (rear-facing) | **measured** |
| barge | — | — | — | — | — | placeholder — the CAD has its mount (front-left, ~40.6" up, 13.4" forward) but no camera body in it; measure, then set `measured = true` |
| reef (front-left, base stage) | +11.04" | −11.25" (left) | 15.78" | −20° (tilted down) | −30° (toward the centerline, i.e. to the right) | **from the CAD** (LL3G lens-barrel axis in the Leviathan STEP) — verify in the web UI |

One-time check after deploying: open `http://limelight-funnel.local:5801`, confirm the 3D preview shows the camera at the back, pointing rearward and tilted up; then `http://limelight-reef.local:5801` should show the camera front-left, low, looking forward-right and down. If either points the wrong way, flip the pitch or yaw sign in the constant. With the camera tilted 50°, the tracker's *Station Flush Distance* is still simply whatever `Vision Distance` reads when the rear bumpers are flush — the tilt is baked into that reading.

**Heat and fan noise.** Two things the code does to keep the Limelights cool without giving up tracking performance: the **LEDs are never turned on** (AprilTags need no illumination, and the LED array is the camera's biggest heat source), and processing is **throttled while the robot is disabled** (one frame processed per 100 skipped — still ~1 solve/s for the pre-match heading seed) with full rate restored the instant it enables. The rest is configured on each camera's web UI (`http://limelight-<name>.local:5801`), where these settings dominate CPU/GPU load and therefore fan speed:

| Setting | Recommendation |
|---|---|
| Pipeline type | **AprilTag** only — make sure no retroreflective/neural pipeline is selected by default |
| Detector downscale | **2** (or 3): halves the detection workload; the full-resolution corner refinement still runs, so accuracy is essentially unchanged at reef distances |
| Capture resolution / FPS | The lowest mode that still detects tags across the field — 640×480 @ 90 fps is plenty; full 1280×960 doubles the load for no benefit on a 5 m reef approach |
| Stream resolution ("Stream" settings) | Lowest offered; the MJPEG encoder for the dashboard feed is pure overhead (only the Setup tab shows all three feeds) |
| Multi-target / 3D solve | Leave **enabled** — MegaTag2 depends on it and it is cheap relative to detection |
| Camera exposure/gain | Shorter exposure lowers sensor heat slightly; keep just enough to detect tags reliably |

If a camera still runs hot with those, check its mounting: the fan intake needs clear airflow, and a camera boxed in next to the Kraken or the roboRIO will run warm regardless of settings.

### LEDs ([LEDs.java](src/main/java/frc/robot/subsystems/LEDs.java)) — **commented out**
There is no CANdle on the robot, so the whole subsystem is commented out (every line of `LEDs.java`, `LEDConstants`, and the three "CANdle disabled" spots in `RobotContainer.java` / `Dashboard.java`); the dashboard's *LEDs/State* field just says so. Restore all of them together when a CANdle is installed. What the code does when enabled — CTRE CANdle, state derived automatically each loop, no commands needed:

| Priority | State | Pattern |
|---|---|---|
| 1 | Disabled | Alliance-colored Larson scanner |
| 2 | Autonomous | Rainbow |
| 3 | Game piece held | Solid green |
| 4 | Vision tracking | Strobing cyan |
| 5 | Endgame (last 20 s) | Strobing yellow |
| 6 | Teleop idle | Solid alliance color |

Set `LEDConstants.LED_COUNT` to match the installed strip (8 onboard LEDs + strip length) and adjust `StripType` in LEDs.java if colors look swapped.

### Telemetry ([Telemetry.java](src/main/java/frc/robot/Telemetry.java))
Publishes drivetrain state to NetworkTables (for AdvantageScope/Elastic) and CTRE SignalLogger (.hoot logs) at the odometry rate, plus Mechanism2d module visualizations.

---

## Dashboard (Elastic)

All dashboard integration goes through **[Elastic](https://frc-elastic.gitbook.io/docs)** and is centralized in [Dashboard.java](src/main/java/frc/robot/Dashboard.java) — subsystems expose getters and publish nothing themselves. (Elastic removed Shuffleboard API support, so this project publishes plain NetworkTables data plus a layout file.)

### Tabs
| Tab | Purpose | Highlights |
|---|---|---|
| **Setup** | Pre/post-match checks | FMS info, battery + CAN health, Alerts, Zero Elevator / Zero CorAl Pivot buttons, sensor status, **all three Limelight camera feeds** with per-camera "sees tag" lights, auto chooser, branch side |
| **Autonomous** | Auto selection & monitoring | Auto chooser, big match timer, **Field widget with live robot pose**, game piece indicator, Alerts |
| **Teleop** | Driving | **Field widget**, match timer, big game-piece box, swerve module widget, vision tracking + **branch side**, elevator/pivot position bars |
| **Testing** | Independent mechanism testing + diagnostics | **Elevator / Pivot / Intake setpoint sliders with Go/Run/Stop buttons**, **Tunables (Robot Preferences) editor**, command scheduler, subsystem widgets, current graphs, vision + CANrange readouts |

Every tab fits **12 × 5 grid cells** (1536 × 640 px at grid size 128): on the drive laptop's display scaling each cell renders ~155 px, so the window shows ~12.9 columns, and a docked Driver Station leaves ~5 rows visible. The layout is generated from a small script that also verifies no widget overlaps or overflows (see the commit history) — to change it, either edit in Elastic and `File → Export Layout`, or regenerate. The robot **switches Elastic to the right tab automatically** on mode changes (disabled → Setup, auto → Autonomous, teleop → Teleop, test → Testing) via ElasticLib ([util/Elastic.java](src/main/java/frc/robot/util/Elastic.java)).

### Testing one mechanism at a time
The Testing tab drives each mechanism **independently** — set the *Elevator Setpoint* slider and press *Elevator Go*, and only the elevator moves; the arm stays exactly where it is (and the operator's manual stick for the other mechanism keeps working). The same goes for *Pivot Setpoint / Pivot Go* and *Intake Speed / Intake Run / Intake Stop*.

These single-mechanism moves still consult the collision model in [Superstructure.java](src/main/java/frc/robot/Superstructure.java), but instead of moving the *other* mechanism out of the way (as the preset buttons do), an unsafe request is simply **refused with a toast notification** that says why — e.g. "Elevator test move refused: 30.0 in is not reachable with the arm at 0 deg — raise the arm first". Put the arm at 100° (Pivot Setpoint 100 → Pivot Go) and the elevator can be run through its full travel on its own; the refusals come from the CAD corridor table.

### Live tuning without redeploying (Tunables)
Empirically measured numbers live in [util/Tunables.java](src/main/java/frc/robot/util/Tunables.java), backed by **WPILib Preferences**: they appear in the Testing tab's *Tunables* widget, edits apply on the next loop (or next button press), and the roboRIO **persists them to disk** — they survive reboots, power cycles, *and* code deploys. The values in `Constants.java` are only the factory defaults; *Reset Tunables to Defaults* restores them. Because stored values survive a deploy, **changing a default in `Constants.java` does nothing on a robot that already has the key stored** — bump `DEFAULTS_VERSION` in `Tunables.java` (which overwrites every tunable once at the next boot) or press *Reset Tunables*.

| Tunable | Default | Used by |
|---|---|---|
| Elevator – Travel Ratio (measured / modeled) | 1.0 | Carriage inches per motor rotation = 0.733 × this (15:1). Confirmed 1.0 by the 20"/40" tape tests; applied only with the carriage at its hard stop while disabled (the encoders re-reference there). See *Calibrating the elevator height* |
| Elevator – Height At Hard Stop (in) | 0.875 | Where the carriage sits, in the base-2×1-top → carriage-2×1-bottom frame, on its hard stop. The encoder is referenced to this value there and the reverse soft limit sits here. Measure it with a tape at rest; applied like the ratio |
| Elevator – kP (duty per in) / kS (V) / kG (V) | 0.1 / 0.2 / 0.6 | Spark MAX position loop + feedforward, re-applied to both controllers the next time the robot is disabled |
| Elevator – kV Scale (× free-speed model) | 1.0 | Multiplies the NEO back-EMF velocity feedforward (which is derived from the travel ratio). REV: overshoot → lower it |
| Elevator – Cruise Velocity (in/s) / Max Acceleration (in/s²) | 50 / 400 | MAXMotion profile on the 15:1 gearing (full travel ≈1.2 s, ≈8.7 V at cruise). The staged superstructure sequences are gated on measured state, so a change here only alters their timing |
| Pivot – Cruise Velocity (deg/s) / Acceleration (deg/s²) / Jerk (deg/s³) | 200 / 300 / 2000 | CorAl Motion Magic profile, re-applied to the TalonFX the next time the robot is disabled. **Softened for the pivot chain's backlash** (acceleration sets how hard the chain catches the arm at the end of a move; jerk sets how abruptly the stop begins). Once the chain is fixed, 240 / 480 / 4800 tracked cleanly. If a coral slips during a swing, lower the acceleration first. Handoff heights follow these too |
| Elevator – Profile Error (in) | 0.3 | How far the carriage may stray from the MAXMotion profile before it is regenerated (not a settling tolerance) |
| CorAl – Coral Detect Distance (m) / Hysteresis (m) | 0.08 / 0.015 | CANrange proximity threshold: detect below threshold − hysteresis, release above threshold + hysteresis. Set the threshold halfway between the empty-claw and coral readings of `CorAl/CANrange Distance`; re-applied while disabled |
| Vision – Reef Flush Distance (m) | 0.45 | L2–L4 + algae reef alignment (camera-read Z with bumpers flush — measure it: push the robot flush, copy `Vision Distance`) |
| Vision – Station Flush Distance (m) | 0.45 | Coral station alignment (rear camera) |
| Vision – L1 Score Distance (m) | 1.0 | L1 standoff |
| Vision – Reef Branch Offset (m) | 0.165 | Left/right branch centering |
| Vision – Tracking Distance / Rotation kP | 1.5 / 0.06 | Tracking aggressiveness |
| Drive – Teleop Speed Scale (0–1) | 0.25 | Fraction of top speed *and* rotation rate at full stick (0.25 = indoor testing; raise toward 1.0 for competition). Stick deadbands scale with it. |

Mechanism contact geometry (tuck / low-box limits) is deliberately **not** a tunable — those are measured physical facts. The CorAl pivot's Phoenix gains are applied at boot and tuned live in Phoenix Tuner X; the elevator's Spark MAX gains *are* tunables (above) because the elevator is the mechanism that needs on-robot calibration most.

### Calibrating the elevator height
Heights are measured from the **top of the base-stage 2×1 to the bottom of the carriage 2×1**, with the middle stage between them. Five tape tests (2", 6", 8", 20", 40" commanded, under three different ratios) all fit one line: **tape = gearing-model travel + ≈0.875"**. Between the 20" and 40" tests the slope is 1.005, so the gearing model (0.244"/motor rotation) is right; the constant is not a scale error but the height the carriage sits at, in that frame, on its hard stop. The encoder is therefore referenced *to* the hard-stop height (*Elevator – Height At Hard Stop*, default 0.875") rather than zeroed, and the reverse soft limit sits there. A closed-loop problem looks different: the **dashboard itself** stops short of the setpoint (it should hold within ~0.1"); at kP 0.3 a 1" rest error would mean ~3.6 V pushing with no motion.

1. **Measure the hard-stop height once:** disabled, carriage resting on its hard stop, tape from base-2×1 top to carriage-2×1 bottom. Enter it as *Elevator – Height At Hard Stop* (Testing tab); it applies while disabled with the carriage at the hard stop and the encoders re-reference to it (an info alert shows while it is waiting). Press *Zero Elevator* (Setup tab) whenever the carriage has been moved by hand; `Elevator/Height` must read the hard-stop height at rest — the *reads below its hard-stop height* alert fires if it reads lower.
2. Arm at 100° (Pivot Setpoint 100 → Pivot Go). Testing tab: Elevator Setpoint **20** → Elevator Go. Let it settle, then read the **dashboard** height and the **tape**.
3. **Dashboard at 20.0, tape off by the same amount at 20" and 40"** → the hard-stop height is wrong; adjust it by the difference. **Tape off by a percentage** (twice as far off at 40" as at 20") → set *Elevator – Travel Ratio* = current × tape ÷ commanded; it should stay at 1.0.
4. **Dashboard itself stops short** → closed loop: raise *kP* (rest error ≈ friction volts ÷ (12 × kP) inches) or *kS*. **Overshoot** → lower *kV Scale* first, then *kS* — REV's MAXMotion tuning order. All re-apply the next time the robot is disabled.
5. Repeat at **40"** — dashboard and tape should agree within ⅛" at both heights. Then run L4 (52.5") once before touching *Cruise Velocity* / *Max Acceleration*.
6. A climb that **stutters and slows down**, especially with the *Elevator sides out of sync* alert showing, means one side is dragging: the follower's soft limits (removed in code — a deploy older than that fix would show exactly this) or a binding stage. `Elevator/Left Current` climbing toward the 50 A limit during a move confirms it; a healthy climb draws a few amps.

### One-time setup on each drive station laptop
1. Install Elastic (2025.2 or newer) and connect to the robot.
2. `File → Load Layout From Robot` (Ctrl+D) and pick `elastic-layout.json` — the robot serves it from the deploy directory over HTTP port 5800 (started in Robot.java with `WebServer.start`).
3. Done — every laptop gets the identical layout. Edit the layout in Elastic, `File → Export Layout`, and commit the exported file back to `src/main/deploy/elastic-layout.json` to update it for everyone.

### Alerts & notifications
Persistent problems (through-bore disconnected, elevator sides out of sync, motor/encoder disagreement, low resting battery) appear in the **Alerts widget** via the WPILib Alerts API; a mid-match through-bore failure additionally fires an Elastic toast notification.

### Logging & analysis (AdvantageKit + AdvantageScope)
Logging runs through **AdvantageKit** (`Robot` extends `LoggedRobot`):

- **DriverStation data, joysticks, and console output** are captured automatically.
- **Structured outputs** are recorded every loop from `Dashboard.update()`: robot `Pose2d`, `ChassisSpeeds`, swerve module states/targets, elevator/arm positions and targets, game-piece state, and the **3D component poses** (below).
- `.wpilog` files land on a USB stick (`/U/logs`) if one is plugged into the roboRIO, otherwise `/home/lvuser/logs` (in simulation: `./logs`). Open them in **AdvantageScope**.
- **Live streaming**: AdvantageScope → *Connect to Robot* with the **RLOG** source on **port 5810** (5800 is taken by the Elastic layout server). NetworkTables live viewing works too — all dashboard topics are plain NT.
- The auto chooser is a `LoggedDashboardChooser`, so every log records which auto was selected.
- CTRE's **SignalLogger** (`.hoot` files) runs alongside for Phoenix signals and **SysId** (open hoot logs in Tuner X or convert for SysId).
- Note: full AdvantageKit **log replay** would additionally require IO-layer hardware abstraction in every subsystem — not done here; this integration provides comprehensive logging, not deterministic replay.

### 3D mechanism animation (CAD in AdvantageScope)
`Dashboard` publishes `RobotState/ComponentPoses` — a `Pose3d[]` of `{middle stage, carriage, arm}` in the robot frame (X forward, Y left, Z up) — every loop, so the elevator and arm animate in 3D from real match logs **and** in simulation. A 2D schematic (`SmartDashboard/Superstructure Mechanism`) is also published for **Glass**/AdvantageScope.

To attach the CAD in AdvantageScope's **3D Field** tab (one-time, per laptop):

1. Export the robot CAD to glTF with one file per moving group — `model.glb` (static chassis), `model_0.glb` (middle stage), `model_1.glb` (carriage), `model_2.glb` (arm).
2. Put them in a `Robot_9143A/` folder next to a `config.json` in AdvantageScope's user-assets directory (*Help → Show Assets Folder*):
   ```json
   {
     "name": "9143A",
     "rotations": [{ "axis": "x", "degrees": 90 }],
     "position": [0, 0, 0],
     "cameras": [],
     "components": [
       { "zeroedRotations": [{ "axis": "x", "degrees": 90 }], "zeroedPosition": [0, 0, 0] },
       { "zeroedRotations": [{ "axis": "x", "degrees": 90 }], "zeroedPosition": [0, 0, 0] },
       { "zeroedRotations": [{ "axis": "x", "degrees": 90 }], "zeroedPosition": [0, 0, 0] }
     ]
   }
   ```
   The `components` order **must** match the `Pose3d[]` order above (index 0 → middle stage, 1 → carriage, 2 → arm). `zeroedPosition`/`zeroedRotations` describe each part's CAD origin at rest — start at zero and adjust until the parts line up (the `x: 90` rotation is the usual glTF Y-up → field Z-up fix).
3. Select the **9143A** model on the 3D robot, then bind its articulated components to the `AdvantageKit/RealOutputs/RobotState/ComponentPoses` field (drag it onto the robot and choose the component/3D mapping). Live-over-NT and log-replay both use the same field.

The mounting offsets in `Dashboard.java` (`ELEVATOR_X_OFFSET`, `ARM_PIVOT_HEIGHT`) are placeholders — VERIFY against the CAD component origins.

### Glass
Everything Glass needs is already on NetworkTables: the `Field` widget (`SmartDashboard/Field`), the superstructure `Mechanism2d`, the command scheduler, all `SmartDashboard` numeric topics for plotting, and the `DriveState` struct topics from `Telemetry`.

### Simulation
`./gradlew simulateJava` starts the full robot in the WPILib **Sim GUI** with physics:

- **Drivetrain**: CTRE's high-fidelity swerve simulation (drive with a gamepad, or the keyboard mapping in `simgui-ds.json`).
- **Elevator and CorAl arm**: `ElevatorSim`/`SingleJointedArmSim` physics wired to the Spark MAX and TalonFX simulation states, so preset buttons move the mechanisms — watch them in the Mechanism2d, the 3D view, or AdvantageScope connected live.
- Vision/Limelights are absent in sim; `getPivotAngle()` automatically falls back to the simulated motor sensor.
- Caveat: REV's *simulated* MAXMotion profile advances on wall-clock time and mis-scales its feedforward, so elevator profile moves overshoot in sim before settling — sim-only as far as the docs say, but verify the first real profile move carefully.

### Unit tests
`./gradlew test` (also run by `build`):

- `RobotContainerTest` — the whole robot wires up in a simulated HAL and the scheduler runs. This test caught a real bug: the CANrange was configured with CAN ID **64**, which is not a legal Phoenix ID and crashed robot code at construction.
- `ElevatorSimTest` — the elevator physics sim drives the carriage and the stick-release hold latches on the controller.
- `gradle.properties` pins Gradle to the **WPILib JDK** — the Android Studio/JetBrains runtime that `JAVA_HOME` points at on this machine ships an old `msvcp140.dll` that hard-crashes WPILib's natives when tests load them. Remove/adjust that line on machines without a `C:/Users/Public/wpilib/2026` install.
- Tests fork one JVM per class (`forkEvery`) because simulated CAN devices reject duplicate IDs within a process.

### Other tools
- **Phoenix Tuner X**: CTRE device config/firmware, hoot log viewing, swerve project generator (note: `tuner-project.json` is stale — see checklist).
- **REV Hardware Client**: Spark MAX firmware and live tuning of the elevator gains.
- **PathPlanner GUI**: edit paths/autos in `src/main/deploy/pathplanner`.
- **Choreo**: draw time-optimal trajectories into `src/main/deploy/choreo`; they appear in the auto chooser automatically (see [Autonomous](#autonomous-pathplanner--choreo)).
- **Elastic**: see the Dashboard section above.
- Optional future addition: **URCL** (Unofficial REV-Compatible Logger) vendordep would stream Spark MAX internals into AdvantageScope; AdvantageKit 26 already exposes `Logger.registerURCL(...)` for it.

---

## Autonomous (PathPlanner + Choreo)

Two path-authoring tools feed the **same** auto chooser (`SmartDashboard/Auto Mode`), so the driver picks a PathPlanner auto **or** a Choreo trajectory from one dropdown.

### PathPlanner
- Autos live in `src/main/deploy/pathplanner/autos`, paths in `.../paths` (2025.X file format, which PathPlanner 2026 uses as well).
- Every auto resets odometry to the path's starting pose (`resetOdom: true`).
- Path constraints: 3 m/s, 4 m/s² (robot max is ~5.96 m/s).
- Path-following feedback gains live in `Constants.AutoConstants` (translation/rotation kP = 5.0). PathPlanner supplies feedforward from the path; these PIDs correct pose error.
- `FollowPathCommand.warmupCommand()` is scheduled at startup so the first path of auto starts without a stutter.
- `deleteOldFiles = true` in build.gradle removes stale paths from the roboRIO on deploy.

### Choreo
- Choreo trajectories live in `src/main/deploy/choreo` (`*.traj`). `RobotContainer.addChoreoAutos()` discovers every `.traj` at startup and adds it to the chooser as **"Choreo: &lt;name&gt;"** — exactly how PathPlanner autos are picked up from their folder. Draw a trajectory in the [Choreo](https://choreo.autos) app (create/open a project whose folder is `src/main/deploy/choreo`) and it appears automatically; a bad file is logged and skipped, never crashing robot code.
- **Following is done by PathPlanner**, not ChoreoLib: PathPlanner 2026 natively loads Choreo `.traj` files (`PathPlannerPath.fromChoreoTrajectory`), so Choreo autos run through the same `AutoBuilder` holonomic controller and `AutoConstants` gains as PathPlanner autos — one code path to tune. Each Choreo auto resets odometry to the trajectory's (alliance-flipped) start pose, then follows.
- **Why not the ChoreoLib vendordep?** ChoreoLib's latest release is 2025 (`frcYear: 2025`); GradleRIO 2026 rejects a mismatched-year vendordep outright (*"...will break at runtime"*), so importing the `.traj` through PathPlanner is the supported way to run Choreo on a 2026 project. Swap to ChoreoLib's own `AutoFactory` if/when a 2026 release ships.
- `Demo.traj` is a placeholder straight-line move (2 m → 3 m on X) so the feature is testable out of the box (`ChoreoTrajectoryTest` loads it through the real parser). Replace it with real trajectories.

---

## Configuration

Constants are grouped by subsystem in [Constants.java](src/main/java/frc/robot/Constants.java): `ElevatorConstants`, `CorAlConstants`, `AutoConstants`, `VisionConstants` (plus the commented-out `LEDConstants`), with a CAN ID / DIO map at the top of the file. Every value is commented with its unit; values marked **TUNE** or **VERIFY** must be confirmed on the robot.

Swerve hardware constants are in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java), generated by CTRE Tuner X.

---

## Geometry confirmed from the CAD

The five STEP exports (`9143-2025-A-0000 Leviathan` and its Drivetrain / Elevator / CorAl / Funnel sub-assemblies) were resolved with a small assembly walker (part placements + bounding boxes + cylinder axes, validated on known tube sizes). CAD frame: X = robot left, Y = up, Z = forward; the floor is Y = 0 (wheel centers sit at exactly 2.000"). What it settled:

| Item | CAD | Code |
|---|---|---|
| Bumper / frame envelope | 36.5" square bumpers on a 30.0" frame, centered on the origin | matches `TunerConstants` / bumper assumptions |
| Swerve steering axes | all four at 12.375" from center — **2.625" inset for the MK4n too**, not just the MK4i | `TunerConstants` ±12.375" ✔ |
| Elevator sprocket | "25 Chain 22t .500 Hex Sprocket" | 22T ✔ |
| Elevator hard-stop height | base bottom cross tube top 5.875", middle-stage 1" tube on it, carriage tube bottom 6.875" → **1.000"** with one stage between | tunable default 0.875" (tape fits 0.87–0.95") — tape at rest decides |
| CorAl pivot axis (hard stop) | through-bore bore: **13.875" up, 12.01" forward** of center | `Dashboard` pivot height 0.352 m, X offset 0.305 m |
| Arm reach | far intake roller axes 13.9" from the pivot; in the CAD pose the arm points 33° past vertical toward the *rear* (the intake pose, facing the funnel) | `Dashboard` arm length 0.352 m |
| Reef Limelight | lens 11.25" left, 15.78" up, 11.04" forward; 20° down, 30° toward the centerline | `LIMELIGHT_POSES[reef]`, pushed to the camera |
| Funnel Limelight | mount centered, ~30" up, 11.6–14.7" behind center | consistent with the hand-measured pose |
| Barge Limelight | mount only, no camera body in the CAD | still a placeholder |

Two things the CAD shows that the code should know about: the carriage stage is rigged with 4 mm Dyneema over pulleys **and constant-force springs** (a gravity assist — expect the tuned elevator kG to land near zero or even slightly negative-feeling at the bottom), and there is a **CANivore** in the brainpan while `TunerConstants` puts the drivetrain on the roboRIO bus (empty bus name) — fine if the CANivore is unused, but if the swerve devices are wired to it the bus name must be set.

### CAD clearance analysis (claw vs. elevator)

The elevator tube contacts were mapped from the CAD instead of guessed: every CorAl part's B-rep edges were sampled (0.2"), projected to the side view about the pivot and filled to the part's true outline; the base stage, middle stage, funnel, frame and bumpers became obstacles (tubes as exact boxes, everything else as dilated edge samples), each tested only against claw parts it overlaps in X; the middle stage rises at half the carriage travel. The map below is the model's clearance for every (arm angle, carriage height): `#` under ¼", `x` under ½", `.` under 1", blank ≥ 1". It reproduces both contacts measured on the robot (0° at ≈10.75", 5° at ≈23") within ½", so treat its figures as *real minus 0 to ½"*.

```
      carriage height (preset frame), one column per 2": 1" ... 53"
      1   9   17  25  33  41  49
    0° |x.   ####.##########x      |
    5° |x.   ##x..x########x       |
   10° |x.        .x######x        |
   15° |x.         .######.        |
   20° |x.         .####x.         |
   25° |x.         .###           x|
   30° |x.         ###.          .#|
   35° |x.         ###          .##|
   40° |x.        x##x          ###|
   45° |x.        ###.         .###|
   50° |x.       .##x         .####|
   55° |x.       .##.         #####|
   60° |x.       xx.         x#####|
   65° |x.       ..         .######|
   70° |x.                  x######|
   75° |x.                 .#######|
   80° |x.                 x######x|
   85° |x.                .######x.|
   90° |#.                .####x.. |
   95° |#.                ......   |
  100° |#.                         |
  105° |#.                         |
  110° |x.                         |
  115° |xx                         |
  120° |xx                         |
  125° |x#                         |
  130° |##                         |
  135° |##                         |
  140° |##.                        |
  145° |##.                        |
  150° |###                        |
  155° |###                        |
  160° |###.                       |
```

Reading it: the blocked wedge from 18–30" for angles under 65° is **band A** (the middle-stage top tube passing the claw's top); the blocked region above ≈36–46" for angles 25–95° is **band B** (the tube and the constant-force-spring hardware, now 1–2" above the pivot, meeting the claw's lower rear) — that is the bar L4 hit, and the reason 90° is not a travel angle; the only clear columns at the top of travel are ≤ 22.5° and ≥ 97°. The bottom-left corner (angles > 130° below ≈7") is the bumper. `SuperstructureConstants.FREE_CORRIDORS` is this map with a 1" margin, and the staged sequences above were simulated against the full map. Reef branches are not in the model: at the L4 angle the claw stays inside the bumper plane (the manual gives branch tips at 31.875"/47.625"/72" inset 1⅝"/1⅝"/1⅛" from the reef face), so only the coral itself reaches the branch.

Scripts (`stepbox.py`, `stepedges.py`, `clearlib.py`, `seqsim2.py`) live outside the repo in the session scratchpad; the inputs are the five STEP exports in Downloads. Re-run them if the claw, the elevator tubes or the spring hardware change.

## Pre-Competition Checklist

1. **Verify the CANcoder offsets in Tuner X** (wheels aligned straight forward). `tuner-project.json` disagrees with `TunerConstants.java` on all four offsets — and note the Tuner project itself is **stale** (it models all four modules as MK4n L3+, but the real robot has MK4i L3 fronts), so do **not** blindly regenerate TunerConstants from it; the mixed per-module ratios now in TunerConstants.java match the physical robot. The MK4n (back) wheel inset is **confirmed from the CAD**: the steering-bearing axes of all four modules sit 12.375" from the frame center (2.625" in from both frame edges, MK4n and MK4i alike), matching `TunerConstants`.
2. **Confirm the elevator hard-stop height** with a tape at rest (base-2×1 top to carriage-2×1 bottom) against *Elevator – Height At Hard Stop* (0.875"), then verify a 20" and a 40" move with the tape — see *Calibrating the elevator height*. The gearing model is confirmed; the travel ratio should stay at 1.0.
3. **Sanity-check the CorAl pivot angle** against the through bore encoder. The 65.41:1 ratio is derived from the real gear train (10:58 → 18:58 → 12:42). Also confirm the through bore is mounted 1:1 on the pivot shaft.
4. **Tune closed-loop gains** (elevator kP/kS/kV/kG and both mechanisms' profiles from the Testing-tab tunables — they re-apply the next time the robot is disabled; pivot kP/kS/kV/kG via Phoenix Tuner X; `AutoConstants` path-following kP) and test autos. The elevator is now **15:1** (50 in/s / 400 in/s², kP 0.1, kG 0.6 — the constant-force springs may want less kG); the pivot ships at 200°/s / 300°/s² / jerk 2000, softened for the chain backlash (240 / 480 / 4800 once the chain is fixed). First moves with the new sequences: watch the **33" L4 station** (the 100°→25° rotation there, then the 20° finish above 37"), the **L4 return drop** to 39" before the arm moves, and the **L3 return lift** to 31". If a held coral slips during a swing, lower the pivot *Acceleration* first.
4a. **The L3 pose (29", 22.5°) is the one preset the CAD model puts inside a contact band** (within ≈¼" of the top sprocket shaft; the dashboard shows an info alert saying so). It works on the robot; if it ever rubs, (25°, 30.5") is the nearest pose with a full inch of clearance.
5. **Tune the vision goal constants**: place the robot flush on the reef base and copy the `Vision/Distance` reading into `REEF_FLUSH_DISTANCE` (same procedure backed up to the coral station for `STATION_FLUSH_DISTANCE`); verify `REEF_BRANCH_OFFSET`, the branch left/right sign, and `LIMELIGHT_FACING_SIGNS` (which cameras face front vs. rear). Verify the L3 approach **at low speed first** — its arm-arrival offset (and the ≈25" handoff it resolves to) is predicted, not measured; adjust *L3 Arm Arrival Offset* on the Testing tab if the mechanism approaches the tube during the rotation.
6. **Firmware**: 2026 firmware on all CTRE devices (TalonFX, CANcoder, Pigeon 2, CANrange, CANdle), current Spark MAX firmware via the REV Hardware Client, 2026 roboRIO image, Limelight OS 2026.0+.
6a. **Set the CANrange's CAN ID to 62 in Tuner X** (or change `CANRANGE_SENSOR_ID` to its actual ID). It was documented as ID 64, which is **not a legal Phoenix ID (0-62)** — robot code crashed at construction with it (caught by the unit tests).
7. **LEDs**: skip while the CANdle code is commented out. When one is installed, un-comment the subsystem and set `LEDConstants.LED_COUNT` to the actual LED strip length.
8. Raise the *Drive – Teleop Speed Scale* tunable on the Testing tab (25% for indoor testing) as the drivers are ready — no redeploy needed.

---

## Dependencies

- **WPILib 2026** (GradleRIO 2026.2.1): The core library for FRC robot programming.
- **Phoenix 6 (v26)**: CTRE motor controllers (Kraken X60/TalonFX) and sensors (CANcoder, CANrange, CANdle, Pigeon 2).
- **REVLib 2026**: Spark MAX controllers and NEO motors on the elevator.
- **PathPlanner 2026**: Autonomous path generation and following — also loads **Choreo** `.traj` files, so Choreo is supported with no extra vendordep (ChoreoLib has no 2026 release yet).
- **AdvantageKit v26**: Logging framework — fully wired (`LoggedRobot`, `.wpilog` + RLOG live stream, logged auto chooser).
- **LimelightHelpers v1.14**: Limelight interface (requires LLOS 2026.0+ on the cameras).

## Getting Started

1. **Clone this repository** to your local machine.
2. **Open the project** in WPILib VS Code 2026.
3. **Build and deploy** (`./gradlew deploy` or the WPILib "Deploy Robot Code" command).
