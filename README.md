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
| Left bumper | Heading fix: **disabled** = re-zero field-centric to alliance-forward; **enabled** = re-seed the heading from AprilTags (MegaTag1 for 2 s). Back + left bumper forces the gyro re-zero while enabled |
| Back/Start + X/Y | SysId characterization (testing only) |

### Operator (Xbox controller, port 1)
All position buttons run coordinated elevator + arm sequences through the **Superstructure**, which automatically avoids mechanism contact from any starting pose.

| Input | Action |
|---|---|
| Left stick Y | Elevator manual control (holds height on release; **no interlocks**) |
| Right stick X | CorAl pivot manual control (holds angle on release; **no interlocks**) |
| D-pad down | Coral **L1** pose (0", 100°) |
| D-pad left | Coral **L2** pose (12", 12.5° — 5° geometric plus chain-backlash sag; a full inch to the cross bar even 2.5° low) |
| D-pad right | Coral **L3** pose (30.5", 25°, rotated within ½" of the target) |
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
Two NEOs on Spark MAX controllers, each through a 15:1 MAXPlanetary reduction (5:1 × 3:1 cartridges — the second 3:1 was removed in Sept 2026 for speed), a 1:1 90° gearbox, and a ½" hex shaft driving 22T #25 sprockets — 5.5" of chain per sprocket rotation, doubled by the cascade rigging. The gearing predicts **0.733" of carriage travel per motor rotation** (0.244" at the earlier 45:1, which 20" and 40" tape tests confirmed to within 0.5%; the *Elevator – Travel Ratio* tunable stays at 1.0 and exists only for a re-check). Heights are measured from the **top of the base-stage 2×1 to the bottom of the carriage 2×1**, and in that frame the carriage sits at **1.000"** when it rests on its hard stop — the gap is the middle-stage tube, a 2×1 on its 1" side (*Elevator – Height At Hard Stop*), so the encoder is referenced *to* that value rather than zeroed — a commanded 20" lands the tape at 20". Travel is 1.0–53" — see *Calibrating the elevator height* below. The right controller is a hardware follower of the left, and encoder conversion factors scale everything to **inches**. The follower carries **no soft limits of its own**: REV documents only that a follower mirrors the leader's voltage output, and since the follower spins opposite the leader its encoder counts negative as the carriage rises — leader limits copied onto it would keep its reverse limit at zero in force for the whole climb and, if enforced, turn it into a brake the leader has to drag (slow, stuttering, current-limited climbs). Height moves use MAXMotion profiles on the controller with on-controller kS/kV/kG feedforward (REVLib 2026), so the carriage tracks smoothly and holds height at targets and when the operator releases the stick. Soft limits bound travel in every control mode, voltage compensation keeps response consistent as the battery sags, and configuration is persisted to flash so a brownout can't revert it.

**Elevator gearing and tuning (15:1, Sept 2026).** The NEO free speed through 15:1 is ≈ 69 in/s at the carriage; cruise is 40 in/s (≈ 6.9 V of kV) with 200 in/s², so full travel takes ≈ 1.5 s and a climb peaks near 9 V, leaving ≈ 3 V for the position loop on a sagging battery.

The first 15:1 values (kP 0.1, no kA) **overshot every target by inches and then hunted**, and the reason is worth keeping written down: at 15:1 one inch of carriage error is only 1.4 motor rotations, so a given duty-per-inch is 3× weaker at the carriage than the same number was at 45:1 — and the acceleration force was being bought entirely with position error. The carriage plus arm plus the reflected rotor inertia is ≈ 23 kg effective, so 300 in/s² needs ≈ 1.2 V, which at kP 0.1 (1.2 V per inch) is a **full inch of lag going up and an inch of lead coming down**. The carriage therefore arrived about an inch past every target, and since that exceeded the 0.3" profile-error window, MAXMotion regenerated the profile from the measured state — with a velocity estimate lagging ≈ 130 ms behind — over and over. Simulating the loop at 1 kHz against the CAD mass reproduces both the old behaviour (11" worst-case overshoot, sustained hunting) and the smooth 45:1 history (0.2"), which is what the current values were chosen against: **kA 0.0040 V per in/s²** carries the ramps, **kP 0.5** restores stiffness (≈ 880 N per inch of error, damping ratio ≈ 0.55), the profile-error window is 0.5" so it only fires on a real disturbance, and the encoder velocity filter is 16 ms × 2 instead of 32 ms × 8. Worst case across the plant uncertainty (mass 10–15 kg, springs carrying 0–60 %, friction, a held game piece, a sagging battery): 0.24" of overshoot and no oscillation.

Nothing counterbalances this elevator, so the motors hold the whole moving mass: ≈ 145 N at the carriage by the CAD estimate, ≈ 9 A per motor, so **kG ships at 1.0 V** (the mass estimate spans 0.8–1.3 V) and should be **measured**: hold a height and read `Elevator/Hold Volts`. One real cost of the lighter gearing: with the robot disabled at height, brake mode lets the carriage creep down ≈ 3 in/s — stow before disabling.

### CorAl ([CorAl.java](src/main/java/frc/robot/subsystems/CorAl.java))
Pivot arm + intake rollers (both Kraken X60). The REV Through Bore absolute encoder is the angle reference: the motor's sensor is seeded from it at startup, before each move, and whenever the arm is idle — never mid-move. Motion Magic in degrees, closed-loop holding at targets. Game piece detection uses the CANrange's on-device proximity bit — *distance below the threshold*, with a hysteresis band on both sides, only while the return is strong enough to be a valid measurement — rejected while the sensor reports a compromised measurement, then debounced 0.3 s on **both** edges; a confirmed arrival stops the intake automatically.

**If the sensor "sees" a coral that isn't there:** the verdict is formed in code from the measured distance, not from the CANrange's own proximity bit — that bit can only mean "closer than the threshold", and if the sensor looks across an empty claw at structure a few centimetres away, it is stuck **on** with nothing held. Two dashboard values set it right: read `CorAl/CANrange Distance` with the claw empty and again with a coral held, put *CorAl – Coral Detect Distance* halfway between the two, and set *CorAl – Coral Detect When Closer (1) Or Farther (0)* to match which way the coral moves the reading (`CorAl/CANrange Detect When` shows which way is active). A reading inside the hysteresis band keeps the previous verdict, a weak or unhealthy return counts as "nothing", and both edges are debounced for 0.3 s. `CorAl/CANrange Raw Detect` is the undebounced verdict and `CorAl/CANrange Signal Strength` the return strength.

### Superstructure ([Superstructure.java](src/main/java/frc/robot/Superstructure.java))
The command factory for all coordinated elevator + CorAl motion. The free region of (elevator height, arm angle) was computed from the robot CAD — every CorAl part's true outline swept about the pivot against the base stage, the middle stage (which rises at *half* the carriage travel), the funnel, frame and bumpers — and validated against the two contacts measured on the robot (0° at ≈10.75", 5° at ≈23", both reproduced within ½"). See *CAD clearance analysis* below for the map. The planner moves through that region in **staged, state-gated steps**: every trigger is a measured height or angle, never a timer, so the sequences stay safe at any elevator or pivot speed — only their duration changes.

**How a move runs.** Every move is planned from the mechanisms' **measured** state when the button is pressed, and runs as staged steps whose handovers are measured heights and angles, never timers — so the sequences are safe at any elevator or pivot speed, and only their duration changes. What the free region forces:

- **Low box.** Any arm angle from 8° up is clear below 16.5", so base→L2 and base→L1 move both mechanisms together. Tucked (< 8°) is allowed only below 8.5" (the cross bar under the top sprockets).
- **Climbing out** needs the arm at ≥ 75° — the middle-stage top tube blocks roughly 18–30" for lower angles — and from **RAISE = 100°** the carriage may go anywhere. **90° is not a travel angle**: the claw's lower rear meets the middle-stage top tube from ≈ 36" up. Ascents give the carriage a ratcheting head start (8.5" while tucked → 16.5" once clear → 36" past 75° → anything once the arm reads RAISE).
- **L3 (30.5", 25°)**: climb at RAISE, rotate within ½" of the target (the 25–30° corridor starts at 30"). Leaving, the carriage lifts clear of the tube and then **descends continuously** as the arm swings up — its floor falls from 30" to nothing as the arm passes 75°, so the lift and the descent are one motion.
- **L4 (52.5", 20°)**: the top of travel is clear only at ≤ 22.5° and the 100°→20° rotation is impossible up there, so the carriage rises to the **33" station** (where 25–100° are all clear) and the arm starts rotating **as the carriage arrives** — the gate is the window, not a settled height. From there the two run **together**: the arm continues down to 20° while the carriage climbs behind it, its ceiling rising as the arm comes down (36" at 95°, 40" at 70°, 48" at 40°, 52.5" at 25°). Leaving mirrors it: drop to 39" at the L4 angle, then the arm sweeps up to RAISE while the carriage keeps descending, its floor falling as the arm rises.
- **Algae poses (160°)** hit the bumper below ≈ 7.5", so the arm waits at 110° until the carriage is above 8" — and, leaving an algae pose for anything low, the arm comes back to RAISE before the carriage drops below 8".
- **Stow** tucks to 0° once the descending carriage passes 8.5".

**Why it is not a step-by-step routine.** The carriage is never commanded to an intermediate height and left to wait there. It is given the **final** height every loop, clamped to what the corridors allow for the angles the arm still has to sweep; as the arm advances the clamp opens and the carriage keeps moving. That costs one walk of the 32-row corridor table per loop, allocation-free, and it cannot deadlock, because the arm's direction within a stage is fixed so the clamp only ever moves in the carriage's favour. Simulated with the profiles the robot runs, the L4 climb from the station takes **0.97 s with the carriage never stopping**, against 1.46 s with 0.82 s of it standing still before.

A route-planning version of this was tried and **reverted**: searching a path over a grid of the corridor map cost millions of lookups inside a command's initialization, which overran the robot loop and made every mechanism worse, and its no-route fallback could end up commanding nothing at all. The lesson is in the commit history — per-loop geometry has to be a table walk, not a search.

`SuperstructureCorridorTest` pins the geometry and the overlaps: the corridor table's shape, the measured contacts, every preset pose, the staged waypoints, and — for each sweep — the carriage target the robot would command at every arm angle, which must keep both the pose and the path to it inside the corridors and must actually arrive.

Everything is decided from **measured** height and angle, never timing, and every move is planned from the current pose — buttons are safe in any order at any time, including pressing a new one mid-move. Manual stick control bypasses these interlocks.

### Vision ([Vision.java](src/main/java/frc/robot/subsystems/Vision.java))
Sends the estimated heading to each Limelight every loop and fuses the returned **MegaTag2** poses into the drivetrain with distance/tag-count-scaled confidence; while **disabled** (and during a driver re-seed) it fuses **MegaTag1** instead, whose solve carries an absolute heading, so the pose heading is field-correct before the match starts. Each camera frame is fused **once** (the NT sample timestamp identifies a frame — re-adding the same sample every loop would collapse the estimate onto the raw camera pose), the cameras' estimates are inserted oldest-first (inserting an older measurement discards newer corrections), and estimates are rejected when off-field, when a MegaTag2 solve averages more than 6 m to its tags, or when a single-tag MegaTag1 solve is ambiguous (> 0.7) or far (> 3 m). Each camera's last fused pose is drawn on the Field widget. Requires Limelight OS 2026.0+.

**Alignment** works in the **robot frame**: each camera's primary tag is converted from Limelight camera space into "where is the tag relative to the robot center" using that camera's mounting pose (lens offset, yaw, pitch — `Vision.tagPositionInRobotFrame`, pinned by `VisionGeometryTest`), so the yawed reef camera and the rear funnel camera drive the same loop with no per-camera mirroring. Three P loops with deadbands and a minimum command: forward error, lateral error (1.5 cm deadband — a coral has only ~3 cm of clearance on a branch), and heading error to the field heading that is **square to the tag's face** (from the 2025 AprilTag layout; the authored paths' end rotations agree with it, which the test checks). Cameras only supply their own tag class (funnel → coral stations, reef → reef, barge → none until its pose is measured), and the first tag chosen is **latched** (and sent to the cameras as the priority tag). While the latched tag is out of view its last sighting is **carried on odometry** for up to 1.5 s, so the goal cannot flip between adjacent reef faces and an approach whose tag leaves the camera at the end still finishes — the rear funnel camera (pitched 50° up) loses the station tag about 0.55 m from the wall, before the bumpers are flush, so that last stretch is dead-reckoned (`Vision/Target From Memory`). `Vision/Forward Error`, `Vision/Lateral Error`, `Vision/Heading Error` and `Vision/Aligned` show the servo state.

**Tracking goals** (where the tag should sit relative to the robot center; 2025 Reefscape tags):
| Tags | Goal |
|---|---|
| Reef (6–11, 17–22), L2–L4 selected | Tag `REEF_FLUSH_DISTANCE` ahead (bumpers flush) and `REEF_BRANCH_OFFSET` to the side, so the robot is centered on the driver-selected **left/right branch**, square to the face |
| Reef, L1 selected | Tag **1 m** ahead, centered, square (room for the arm to swing to 100°) |
| Reef, algae/other | Flush and centered, square |
| Coral stations (1, 2, 12, 13) | Tag `STATION_FLUSH_DISTANCE` **behind** the robot (rear bumpers flush), centered, square — approached backward via the rear funnel camera |
| Barge (4, 5, 14, 15) + processor (3, 16) | Intentionally blank — the tracker ignores them (they still feed MegaTag) |

The flush distances are geometry — half the bumper-to-bumper length (0.464 m) plus a little standoff, 0.47 m by default — and the dashboard's `Vision/Distance` (forward, negative behind) and `Vision/Lateral` (positive left) report the same robot-frame numbers, so tune by pushing the robot into position and copying them.

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
| Elevator – Height At Hard Stop (in) | 1.000 | Where the carriage sits, in the base-2×1-top → carriage-2×1-bottom frame, on its hard stop: the gap is the middle-stage tube (1"). The encoder is referenced to this value there and the reverse soft limit sits here; applied like the ratio |
| Elevator – kP (duty per in) / kS (V) / kG (V) | 0.5 / 0.2 / 1.0 | Spark MAX position loop + feedforward, re-applied to both controllers the next time the robot is disabled. Set kG from `Elevator/Hold Volts` |
| Elevator – kV Scale (× free-speed model) | 1.0 | Multiplies the NEO back-EMF velocity feedforward (which is derived from the travel ratio). REV: overshoot → lower it |
| Elevator – kA (V per in/s²) | 0.0040 | Acceleration feedforward, applied by MAXMotion on the profile's acceleration. Lags on the ramps → raise; leads into the target → lower |
| Elevator – Cruise Velocity (in/s) / Max Acceleration (in/s²) | 40 / 200 | MAXMotion profile on the 15:1 gearing (full travel ≈ 1.5 s, ≈ 6.9 V at cruise). Acceleration is limited by how hard the mast rings when the profile stops, not by the motors — 300 shook it visibly, 200 costs 0.07 s. The staged superstructure sequences are gated on measured state, so a change here only alters their timing |
| Pivot – Cruise Velocity (deg/s) / Acceleration (deg/s²) / Jerk (deg/s³) | 200 / 300 / 2000 | CorAl Motion Magic profile, re-applied to the TalonFX the next time the robot is disabled. **Softened for the pivot chain's backlash** (acceleration sets how hard the chain catches the arm at the end of a move; jerk sets how abruptly the stop begins). Once the chain is fixed, 240 / 480 / 4800 tracked cleanly. If a coral slips during a swing, lower the acceleration first. Handoff heights follow these too |
| Elevator – Profile Error (in) | 0.5 | How far the carriage may stray from the MAXMotion profile before it is regenerated from the measured state (not a settling tolerance). Small values fire on every acceleration ramp |
| CorAl – Coral Detect Distance (m) / Hysteresis (m) | 0.08 / 0.015 | CANrange proximity threshold: detect below threshold − hysteresis, release above threshold + hysteresis. Set the threshold halfway between the empty-claw and coral readings of `CorAl/CANrange Distance`; re-applied while disabled |
| CorAl – Coral Detect When Closer (1) Or Farther (0) | 1 | Which side of the threshold a held coral puts the reading on. Set it from `CorAl/CANrange Distance` empty vs. holding a coral |
| Vision – Reef Flush Distance (m) | 0.45 | L2–L4 + algae reef alignment (camera-read Z with bumpers flush — measure it: push the robot flush, copy `Vision Distance`) |
| Vision – Station Flush Distance (m) | 0.45 | Coral station alignment (rear camera) |
| Vision – L1 Score Distance (m) | 1.0 | L1 standoff |
| Vision – Reef Branch Offset (m) | 0.165 | Left/right branch centering |
| Vision – Tracking Distance / Rotation kP | 1.5 / 0.06 | Tracking aggressiveness |
| Drive – Teleop Speed Scale (0–1) | 0.25 | Fraction of top speed *and* rotation rate at full stick (0.25 = indoor testing; raise toward 1.0 for competition). Stick deadbands scale with it. |

**At-target tolerances** (not tunables — they are what "arrived" means): the elevator is **0.05"** (about three encoder counts; the tuned loop rests within ≈ 0.02") and the CorAl pivot is **1.0°** measured on the through bore, which is 0.27" at the claw. Because the pivot's closed loop runs on the motor sensor while the tolerance is checked on the through bore, that window also has to cover the chain's backlash — so the Superstructure's settle ends when both mechanisms are at target **or** have stopped moving, whichever comes first. That is what makes a tight window safe: once motion has ceased, the loops are holding their latched setpoints and waiting longer cannot improve the pose. A separate 0.25" threshold (one skipped chain tooth) drives the elevator's out-of-sync alert.

Mechanism contact geometry (tuck / low-box limits) is deliberately **not** a tunable — those are measured physical facts. The CorAl pivot's Phoenix gains are applied at boot and tuned live in Phoenix Tuner X; the elevator's Spark MAX gains *are* tunables (above) because the elevator is the mechanism that needs on-robot calibration most.

### Calibrating the elevator height
Heights are measured from the **top of the base-stage 2×1 to the bottom of the carriage 2×1**, with the middle stage between them. Five tape tests (2", 6", 8", 20", 40" commanded, under three different ratios) all fit one line: **tape = gearing-model travel + a constant**. Between the 20" and 40" tests the slope is 1.005, so the gearing model (0.244"/motor rotation) is right; the constant is not a scale error but the height the carriage sits at, in that frame, on its hard stop — and that height is simply the **middle-stage tube, 1.000"** (the tape readings of ≈0.875" were an eighth of an inch optimistic). The encoder is therefore referenced *to* the hard-stop height (*Elevator – Height At Hard Stop*, default 1.000") rather than zeroed, and the reverse soft limit sits there. A closed-loop problem looks different: the **dashboard itself** stops short of the setpoint (it should hold within ~0.1"); at kP 0.3 a 1" rest error would mean ~3.6 V pushing with no motion.

1. **Measure the hard-stop height once:** disabled, carriage resting on its hard stop, tape from base-2×1 top to carriage-2×1 bottom. Enter it as *Elevator – Height At Hard Stop* (Testing tab); it applies while disabled with the carriage at the hard stop and the encoders re-reference to it (an info alert shows while it is waiting). Press *Zero Elevator* (Setup tab) whenever the carriage has been moved by hand; `Elevator/Height` must read the hard-stop height at rest — the *reads below its hard-stop height* alert fires if it reads lower.
2. Arm at 100° (Pivot Setpoint 100 → Pivot Go). Testing tab: Elevator Setpoint **20** → Elevator Go. Let it settle, then read the **dashboard** height and the **tape**.
3. **Dashboard at 20.0, tape off by the same amount at 20" and 40"** → the hard-stop height is wrong; adjust it by the difference. **Tape off by a percentage** (twice as far off at 40" as at 20") → set *Elevator – Travel Ratio* = current × tape ÷ commanded; it should stay at 1.0.
4. **Dashboard itself stops short** → closed loop: raise *kP* (rest error ≈ friction volts ÷ (12 × kP) inches) or *kS*. **Overshoot** → lower *kV Scale* first, then *kS* — REV's MAXMotion tuning order. All re-apply the next time the robot is disabled.
5. Repeat at **40"** — dashboard and tape should agree within ⅛" at both heights. Then run L4 (52.5") once before touching *Cruise Velocity* / *Max Acceleration*.
5a. **If the elevator overshoots or hunts after a change**, work in this order: (1) hold a height and read `Elevator/Hold Volts` — that is what kG should be; (2) watch `Elevator/Velocity` against the commanded cruise during a long move: if the carriage is still short of cruise when the profile says it should be there, raise *kA*; (3) if it rests short of the target, raise *kP* (or fix kG); if it buzzes at rest, lower *kP*; (4) only then touch the profile. Overshoot that grows with the move length is kA; overshoot that is the same at every target is kP/kG.
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
- `.wpilog` files land on a USB stick (`/U/logs`) if one is mounted on the roboRIO, otherwise `/home/lvuser/logs`, where the oldest logs are pruned to keep 100 MB free and the dashboard shows a warning (in simulation: `./logs`). Open them in **AdvantageScope**. Every logged output is also published live over NetworkTables under `/AdvantageKit` (NT4Publisher).
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
- Every auto resets odometry to the path's starting pose (`resetOdom: true`) through `Swerve.resetPoseForAuto`, which **keeps the vision-seeded heading** (resetting only the translation) when a two-or-more-tag MegaTag1 solve was fused in the last 3 s, the estimator heading has converged on it (within 3° — one fused solve only closes part of the error), and it agrees with the nominal heading within 20°; otherwise the full nominal pose is used. `Vision/Auto Kept Heading` shows which happened. (MegaTag2 trusts that heading absolutely for the whole period, so overwriting a good one with the nominal placement would bias every vision pose.)
- The match autos (`Left Wall - 3 Piece`, `Right Wall - 3 Piece`) run **named commands** between paths: `scoreL4` (raise, eject, stow) at each reef face and `intakeCoral` (stow, rollers until the CANrange confirms, 3 s timeout) at each station; `scoreL3`, `scoreL2` and `stow` are registered too for the PathPlanner GUI. Sequenced, three L4 cycles take well over 15 s — trim in the GUI once real cycle times are known. `Practice - Two Piece ...` start at a coral station (not a legal match start) and `Test - ...` are drivetrain checks: neither belongs in a match.
- `FlippingUtil` is set to the 2025 field (17.548 × 8.052 m) before AutoBuilder is configured — PathPlannerLib 2026 otherwise mirrors red-alliance paths, odometry resets and Choreo start poses about the 2026 field's centerline, about 1 m off.
- If `settings.json` cannot be loaded, the chooser offers only `None` and an error alert says so, instead of the robot program crashing on the chooser build.
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
| Elevator hard-stop height | base bottom cross tube top 5.875", middle-stage 1" tube on it, carriage tube bottom 6.875" → **1.000"** with one stage between | **1.000"** — confirmed by the team (the gap is the middle-stage tube); the clearance map is computed at the same rest height |
| CorAl pivot axis (hard stop) | through-bore bore: **13.875" up, 12.01" forward** of center | `Dashboard` pivot height 0.352 m, X offset 0.305 m |
| Arm reach | far intake roller axes 13.9" from the pivot; in the CAD pose the arm points 33° past vertical toward the *rear* (the intake pose, facing the funnel) | `Dashboard` arm length 0.352 m |
| Reef Limelight | lens 11.25" left, 15.78" up, 11.04" forward; 20° down, 30° toward the centerline | `LIMELIGHT_POSES[reef]`, pushed to the camera |
| Funnel Limelight | mount centered, ~30" up, 11.6–14.7" behind center | consistent with the hand-measured pose |
| Barge Limelight | mount only, no camera body in the CAD | still a placeholder |

Two things the CAD shows that the code should know about: the carriage stage is rigged with 4 mm Dyneema over pulleys (the CAD also shows **constant-force springs** on the middle stage — **those are not on the robot**, that part of the CAD is stale, so the elevator has no gravity assist and kG is the full-weight value), and there is a **CANivore** in the brainpan while `TunerConstants` puts the drivetrain on the roboRIO bus (empty bus name) — fine if the CANivore is unused, but if the swerve devices are wired to it the bus name must be set.

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

Reading it: the blocked wedge from 18–30" for angles under 65° is **band A** (the middle-stage top tube passing the claw's top); the blocked region above ≈36–46" for angles 25–95° is **band B** (the tube and the spring-mount hardware, now 1–2" above the pivot, meeting the claw's lower rear — the springs themselves are not installed, so if their brackets came off too this band is looser than the map says; the map is the conservative case) — that is the bar L4 hit, and the reason 90° is not a travel angle; the only clear columns at the top of travel are ≤ 22.5° and ≥ 97°. The bottom-left corner (angles > 130° below ≈7") is the bumper. `SuperstructureConstants.FREE_CORRIDORS` is this map with a 1" margin, and the staged sequences above were simulated against the full map. Reef branches are not in the model: at the L4 angle the claw stays inside the bumper plane (the manual gives branch tips at 31.875"/47.625"/72" inset 1⅝"/1⅝"/1⅛" from the reef face), so only the coral itself reaches the branch.

Scripts (`stepbox.py`, `stepedges.py`, `clearlib.py`, `seqsim2.py`) live outside the repo in the session scratchpad; the inputs are the five STEP exports in Downloads. Re-run them if the claw, the elevator tubes or the spring hardware change.

## Pre-Competition Checklist

1. **Verify the CANcoder offsets in Tuner X** (wheels aligned straight forward). `tuner-project.json` disagrees with `TunerConstants.java` on all four offsets — and note the Tuner project itself is **stale** (it models all four modules as MK4n L3+, but the real robot has MK4i L3 fronts), so do **not** blindly regenerate TunerConstants from it; the mixed per-module ratios now in TunerConstants.java match the physical robot. The MK4n (back) wheel inset is **confirmed from the CAD**: the steering-bearing axes of all four modules sit 12.375" from the frame center (2.625" in from both frame edges, MK4n and MK4i alike), matching `TunerConstants`.
2. **The elevator hard-stop height is 1.000"** (the middle-stage tube) — *Elevator – Height At Hard Stop*. Verify a 20" and a 40" move with a tape — see *Calibrating the elevator height*. The gearing model is confirmed; the travel ratio should stay at 1.0.
3. **Sanity-check the CorAl pivot angle** against the through bore encoder. The 65.41:1 ratio is derived from the real gear train (10:58 → 18:58 → 12:42). Also confirm the through bore is mounted 1:1 on the pivot shaft.
4. **Tune closed-loop gains** (elevator kP/kS/kV/kG and both mechanisms' profiles from the Testing-tab tunables — they re-apply the next time the robot is disabled; pivot kP/kS/kV/kG via Phoenix Tuner X; `AutoConstants` path-following kP) and test autos. The elevator is now **15:1** and retuned for it (40 in/s / 200 in/s², kP 0.5, kA 0.0040, kG 1.0 — measure kG from `Elevator/Hold Volts`; nothing counterbalances the carriage); the pivot ships at 200°/s / 300°/s² / jerk 2000, softened for the chain backlash (240 / 480 / 4800 once the chain is fixed). First moves with the new sequences: watch the **33" L4 station** (the 100°→25° rotation there, then the 20° finish above 37"), the **L4 return drop** to 39" before the arm moves, and the **L3 return lift** to 31". If a held coral slips during a swing, lower the pivot *Acceleration* first.
4a. **Every preset is now inside the CAD corridors.** The L3 pose moved from (29", 22.5°) — within ≈¼" of the top sprocket shaft in the model — to (30.5", 25°): the claw tip sits 0.7" further forward and 1.6" higher, so re-check the coral's landing on the L3 branch; if it ever rubs, 27.5° at 30.5" has 1¾". L2 went from 10° to 12.5° so the arm keeps a full inch to the cross bar even when the chain lets it sag 2.5°.
5. **Tune the vision goal constants**: push the robot flush on the reef base and copy `Vision/Distance` into *Vision – Reef Flush Distance*; center it on a branch and copy the magnitude of `Vision/Lateral` into *Reef Branch Offset*; for the station, note that the funnel camera loses the tag about 0.55 m from the wall (its +50° pitch), so read `Vision/Distance` at the last position it still shows a tag and keep *Station Flush Distance* at the geometric 0.47 m — the tracker finishes the last stretch on odometry. All readouts are robot-frame, so the camera that sees the tag does not matter, and a pasted negative value is taken by magnitude. The alignment servo is **untested on the robot**: first runs at a low *Tracking Distance kP* with the mechanism stowed, watching `Vision/Forward Error`, `Vision/Lateral Error`, `Vision/Heading Error` and `Vision/Aligned` — if an axis drives the wrong way, the camera pose (not a sign flag) is what to check, in the camera's web-UI 3D preview.
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
