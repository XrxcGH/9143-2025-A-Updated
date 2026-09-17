# Team 9143 - 2025 Robot Code (2026 Libraries)

This repository contains the code for Team 9143's 2025 FRC robot, updated to the 2026 WPILib/vendor libraries. The drivetrain and CorAl run Kraken X60s through CTRE Phoenix 6; the elevator runs NEOs on Spark MAX controllers through REVLib.

---

## Robot Overview

- **Swerve Drivetrain**: CTRE Phoenix 6 swerve on a 30"×30" frame — mixed modules (front: SDS MK4i, back: SDS MK4n), **all with L3+ drive gearing**, all Kraken X60 drive/steer, CANcoders, and a Pigeon 2. Drive/coupling ratios are uniform; only the steer ratios differ and are set per module.
- **Elevator Subsystem**: Dual NEO/Spark MAX elevator (45:1 MAXPlanetary) with MAXMotion height control in inches.
- **CorAl (Coral and Algae) Subsystem**: Pivoting arm + intake rollers with absolute-encoder-referenced angle control and CANrange game piece detection.
- **Vision System**: Three Limelights fusing MegaTag2 pose estimates into odometry, plus AprilTag tracking.
- **LED System**: CTRE CANdle showing robot state (alliance, game piece, vision tracking, endgame).
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
| D-pad left | Coral **L2** pose (12", 5°) |
| D-pad right | Coral **L3** pose (29", 22.5°, handoff overlap) |
| D-pad up | Coral **L4** pose (52.5", 45°, handoff overlap) |
| A | Coral intake (stow to base, rollers until CANrange confirms) |
| X | Eject coral at the current pose (rollers 0.5 s) |
| B | Algae **low** intake (20.5", 160°, rollers in) |
| Back | Algae **high** intake (37.5", 160°, rollers in) |
| Y | Algae hold (hold rollers, arm to 90°) |
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
Two NEOs on Spark MAX controllers, each through a 45:1 MAXPlanetary reduction (5:1 × 3:1 × 3:1 cartridges), a 1:1 90° gearbox, and a ½" hex shaft driving 22T #25 sprockets — 5.5" of chain per sprocket rotation, doubled by the cascade rigging. The gearing alone predicts 0.244" of carriage travel per motor rotation; the applied conversion multiplies that by the *Elevator – Travel Ratio* tunable (default 1.17 → **≈0.287" per motor rotation**), which is calibrated on the robot with a tape measure — see *Calibrating the elevator height* below. Travel is 0–53" measured from the bottom of the carriage 2×1 to the top of the base-stage 2×1. The right controller is a hardware follower of the left, and encoder conversion factors scale everything to **inches**. Height moves use MAXMotion profiles on the controller with on-controller kS/kV/kG feedforward (REVLib 2026), so the carriage tracks smoothly and holds height at targets and when the operator releases the stick. Soft limits bound travel in every control mode, voltage compensation keeps response consistent as the battery sags, and configuration is persisted to flash so a brownout can't revert it.

### CorAl ([CorAl.java](src/main/java/frc/robot/subsystems/CorAl.java))
Pivot arm + intake rollers (both Kraken X60). The REV Through Bore absolute encoder is the angle reference: the motor's sensor is seeded from it at startup, before each move, and whenever the arm is idle — never mid-move. Motion Magic in degrees, closed-loop holding at targets. Game piece detection uses the CANrange's on-device proximity bit with a 0.3 s rising-edge debounce, and stops the intake automatically.

### Superstructure ([Superstructure.java](src/main/java/frc/robot/Superstructure.java))
The command factory for all coordinated elevator + CorAl motion. The arm (0–160°) sweeps through the elevator's structure, and three contact combinations were measured on the robot: the tucked arm (0°) hits a static elevator part at 10.75" of height; a 5° arm hits the second-stage tube passing 23"; and at full height the arm cannot rotate between 90° and 45° in place.

A **motion planner** builds each move from the real mechanism state at the moment the button is pressed, picking the fastest sequence that stays inside the safe regions:

- **Low-box moves run direct.** Below `LOW_TRAVEL_MAX_HEIGHT` (21") with the arm at/above `ARM_CLEAR_MIN_ANGLE` (5°), nothing contacts — so base→L2, base→L1, and base→algae-low move the arm and elevator together with **no 90° excursion**.
- **Leaving the low box travels at the safe angle (90°)**, but ascents give the elevator a **state-gated head start** (up to the highest height safe for the current arm angle) while the arm swings up, so the swing costs little or no time.
- **Handoff overlaps** — above the low box the arm can't rotate below 90° in place without sweeping into the second-stage tube, so the final rotation overlaps the last part of the climb for L3 (29", 22.5°) and L4 (52.5", 45°). The height where the rotation starts is **derived at plan time from both mechanisms' motion profiles** (pivot swing time vs. elevator decel/cruise) and one tunable per pose — the *arm arrival offset*: how many seconds after the elevator settles the arm finishes its rotation (negative = arm finishes early). Retuning the elevator or pivot speed therefore keeps the overlap in sync automatically; the resolved heights (≈25" for L3 and ≈40" for L4 with the shipped profiles) show live on the Testing tab. Leaving those poses mirrors the overlap with the arm swinging up during the initial descent.
- **Stow overlaps the tuck** — the arm starts rotating to 0° as soon as the descending carriage passes the 10" tuck limit.

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
| barge | — | — | — | — | — | placeholder — measure, then set `measured = true` |
| reef | — | — | — | — | — | placeholder — measure, then set `measured = true` |

One-time check after deploying: open `http://limelight-funnel.local:5801`, confirm the 3D preview shows the camera at the back, pointing rearward and tilted up. If it points the wrong way, flip the pitch or yaw sign in the constant. With the camera tilted 50°, the tracker's *Station Flush Distance* is still simply whatever `Vision Distance` reads when the rear bumpers are flush — the tilt is baked into that reading.

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

### LEDs ([LEDs.java](src/main/java/frc/robot/subsystems/LEDs.java))
CTRE CANdle. State is derived automatically each loop — no commands needed:

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

These single-mechanism moves still consult the collision model in [Superstructure.java](src/main/java/frc/robot/Superstructure.java), but instead of moving the *other* mechanism out of the way (as the preset buttons do), an unsafe request is simply **refused with a toast notification** that says why — e.g. "Elevator test move refused: 30.0 in is not reachable with the arm at 0 deg — raise the arm first". Put the arm at 90° (Pivot Setpoint 90 → Pivot Go) and the elevator can be run through its full travel on its own.

### Live tuning without redeploying (Tunables)
Empirically measured numbers live in [util/Tunables.java](src/main/java/frc/robot/util/Tunables.java), backed by **WPILib Preferences**: they appear in the Testing tab's *Tunables* widget, edits apply on the next loop (or next button press), and the roboRIO **persists them to disk** — they survive reboots, power cycles, *and* code deploys. The values in `Constants.java` are only the factory defaults; *Reset Tunables to Defaults* restores them.

| Tunable | Default | Used by |
|---|---|---|
| Elevator – Travel Ratio (measured / modeled) | 1.17 | Carriage inches per motor rotation = 0.244 × this. Applied only with the carriage at its base while disabled (the encoders re-zero there). See *Calibrating the elevator height* |
| Elevator – kP (duty per in) / kS (V) / kG (V) | 0.3 / 0 / 0.35 | Spark MAX position loop + feedforward, re-applied to both controllers the next time the robot is disabled |
| Elevator – kV Scale (× free-speed model) | 1.0 | Multiplies the NEO back-EMF velocity feedforward (which is derived from the travel ratio). REV: overshoot → lower it |
| Elevator – Cruise Velocity (in/s) / Max Acceleration (in/s²) | 8 / 24 | MAXMotion profile; the Superstructure handoff heights follow these automatically |
| Elevator – Profile Error (in) | 0.3 | How far the carriage may stray from the MAXMotion profile before it is regenerated (not a settling tolerance) |
| Vision – Reef Flush Distance (m) | 0.45 | L2–L4 + algae reef alignment (camera-read Z with bumpers flush — measure it: push the robot flush, copy `Vision Distance`) |
| Vision – Station Flush Distance (m) | 0.45 | Coral station alignment (rear camera) |
| Vision – L1 Score Distance (m) | 1.0 | L1 standoff |
| Vision – Reef Branch Offset (m) | 0.165 | Left/right branch centering |
| Vision – Tracking Distance / Rotation kP | 1.5 / 0.06 | Tracking aggressiveness |
| Superstructure – L3 / L4 Arm Arrival Offset (s) | +1.15 / −0.30 | Seconds after the elevator settles that the arm finishes its rotation (negative = early). The handoff *heights* are derived from these and the motion profiles — see the Testing tab's *Handoffs* readout |
| Drive – Teleop Speed Scale (0–1) | 0.25 | Fraction of top speed *and* rotation rate at full stick (0.25 = indoor testing; raise toward 1.0 for competition). Stick deadbands scale with it. |

Mechanism contact geometry (tuck / low-box limits) is deliberately **not** a tunable — those are measured physical facts. The CorAl pivot's Phoenix gains are applied at boot and tuned live in Phoenix Tuner X; the elevator's Spark MAX gains *are* tunables (above) because the elevator is the mechanism that needs on-robot calibration most.

### Calibrating the elevator height (travel ratio)
A commanded height only means something if the encoder→inches conversion is right, and the two on-robot tape measurements so far **disagree**: 2" commanded → ~2.75" traveled, then (after correcting for that) 6" commanded → 5.125" traveled, with a visible overshoot before it settled. Both readings fit *either* a ≈1.17 scale error *or* the gearing model being exactly right with the carriage starting ~¾" **below the encoder's zero** (zeroed with the carriage raised, then dropped onto the hard stop). The closed loop cannot be what leaves it short: at kP 0.3 a 0.875" rest error would mean the Spark MAX was pushing ~3 V upward and the carriage not moving. This procedure separates the two cases in one move:

1. **Disabled, carriage resting on its hard stop:** press *Zero Elevator* (Setup tab) and confirm `Elevator/Height` reads **0.0**. If it ever reads negative at rest, the *Elevator reads below zero* alert fires — re-zero before trusting any move.
2. Arm at 90° (Pivot Setpoint 90 → Pivot Go). Testing tab: Elevator Setpoint **20** → Elevator Go. Let it settle, then read two things: the **dashboard** height and a **tape measure** of how far the carriage actually rose.
3. **Dashboard reads ≈20.0 but the tape doesn't** → it's the conversion. Set *Elevator – Travel Ratio* = current ratio × tape ÷ 20 (e.g. 1.17 × 17.1 ÷ 20 = 1.00). Lower the carriage to its base and disable: the new ratio applies and re-zeros automatically (an info alert shows while it is waiting).
4. **Dashboard itself stops short of 20.0** (it should hold within ~0.1") → it's the closed loop, not the conversion. Raise *kP* (rest error ≈ friction volts ÷ (12 × kP) inches) or add *kS*. If it **overshoots**, lower *kV Scale* first, then *kS* — REV's MAXMotion tuning order. Every one of these re-applies the next time the robot is disabled.
5. Repeat the 20" move — dashboard and tape should agree within ⅛". Then run L4 (52.5") once at the shipped gentle profile before raising *Cruise Velocity* / *Max Acceleration*.

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

Constants are grouped by subsystem in [Constants.java](src/main/java/frc/robot/Constants.java): `ElevatorConstants`, `CorAlConstants`, `LEDConstants`, `AutoConstants`, `VisionConstants`, with a CAN ID / DIO map at the top of the file. Every value is commented with its unit; values marked **TUNE** or **VERIFY** must be confirmed on the robot.

Swerve hardware constants are in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java), generated by CTRE Tuner X.

---

## Pre-Competition Checklist

1. **Verify the CANcoder offsets in Tuner X** (wheels aligned straight forward). `tuner-project.json` disagrees with `TunerConstants.java` on all four offsets — and note the Tuner project itself is **stale** (it models all four modules as MK4n L3+, but the real robot has MK4i L3 fronts), so do **not** blindly regenerate TunerConstants from it; the mixed per-module ratios now in TunerConstants.java match the physical robot. Also confirm the MK4n (back) wheel inset from the SDS layout drawing — the code assumes 2.625" from the frame edge like the MK4i.
2. **Calibrate the elevator travel ratio** with the 20" procedure in *Calibrating the elevator height* (Testing tab, no redeploy). The shipped default (1.17) trusts the 6" measurement; the earlier 2" reading disagreed with it, so confirm the dashboard reads 0.0 at the hard stop before trusting any move — a ¾" zero offset explains both readings with the gearing model untouched.
3. **Sanity-check the CorAl pivot angle** against the through bore encoder. The 65.41:1 ratio is derived from the real gear train (10:58 → 18:58 → 12:42). Also confirm the through bore is mounted 1:1 on the pivot shaft.
4. **Tune closed-loop gains** (elevator kP/kS/kV/kG and profile from the Testing-tab tunables — they re-apply the next time the robot is disabled; pivot kP/kS/kV/kG via Phoenix Tuner X; `AutoConstants` path-following kP) and test autos. The shipped motion profiles are deliberately **gentle first-power-on values** (elevator 8 in/s, pivot 60°/s with a jerk limit and a 60 A stator cap) so nothing can slam while gains are dialed in — raise them once holding and tracking are clean (elevator: *Cruise Velocity* / *Max Acceleration* tunables, no redeploy; pivot: `Constants.java`; targets are noted in the comments).
5. **Tune the vision goal constants**: place the robot flush on the reef base and copy the `Vision/Distance` reading into `REEF_FLUSH_DISTANCE` (same procedure backed up to the coral station for `STATION_FLUSH_DISTANCE`); verify `REEF_BRANCH_OFFSET`, the branch left/right sign, and `LIMELIGHT_FACING_SIGNS` (which cameras face front vs. rear). Verify the L3 approach **at low speed first** — its arm-arrival offset (and the ≈25" handoff it resolves to) is predicted, not measured; adjust *L3 Arm Arrival Offset* on the Testing tab if the mechanism approaches the tube during the rotation.
6. **Firmware**: 2026 firmware on all CTRE devices (TalonFX, CANcoder, Pigeon 2, CANrange, CANdle), current Spark MAX firmware via the REV Hardware Client, 2026 roboRIO image, Limelight OS 2026.0+.
6a. **Set the CANrange's CAN ID to 62 in Tuner X** (or change `CANRANGE_SENSOR_ID` to its actual ID). It was documented as ID 64, which is **not a legal Phoenix ID (0-62)** — robot code crashed at construction with it (caught by the unit tests).
7. **Set `LEDConstants.LED_COUNT`** to the actual LED strip length.
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
