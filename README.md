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
| Left stick | Field-centric translation (currently scaled to 25% for practice) |
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
| Left bumper | Reset elevator encoders (works disabled) |
| Start | Reset CorAl pivot encoder (works disabled) |

---

## Subsystems

### Swerve ([Swerve.java](src/main/java/frc/robot/subsystems/Swerve.java))
Extends the Phoenix 6 `SwerveDrivetrain` (high-frequency odometry and module control run in CTRE's code). Adds PathPlanner `AutoBuilder` configuration, vision pose fusion (with the required FPGA-to-Phoenix timestamp conversion), an AprilTag tracking command, SysId routines, and alliance-aware operator perspective.

**Mixed modules**: the front pair are SDS MK4i and the back pair SDS MK4n, both with **L3+ drive gearing** (5.36:1, 16T pinion) — so drive and coupling ratios are identical on all four modules, and only the **steer ratios differ** (MK4i 150/7:1 front, MK4n 18.75:1 back), set per module in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java). All driving (teleop and path following) uses **closed-loop velocity** so wheel speeds track the request regardless of battery sag. Top speed ~5.96 m/s at 12 V, matched by PathPlanner's `maxDriveSpeed`.

### Elevator ([Elevator.java](src/main/java/frc/robot/subsystems/Elevator.java))
Two NEOs on Spark MAX controllers, each through a 45:1 MAXPlanetary reduction (5:1 × 3:1 × 3:1 cartridges), a 1:1 90° gearbox, and a ½" hex shaft driving 22T #25 sprockets — 5.5" of chain per sprocket rotation, doubled to **11" of carriage travel** by the cascade rigging (≈0.244" per motor rotation). Travel is 0–53" measured from the bottom of the carriage 2×1 to the top of the base-stage 2×1. The right controller is a hardware follower of the left, and encoder conversion factors scale everything to **inches**. Height moves use MAXMotion profiles on the controller with on-controller kS/kV/kG feedforward (REVLib 2026), so the carriage tracks smoothly and holds height at targets and when the operator releases the stick. Soft limits bound travel in every control mode, voltage compensation keeps response consistent as the battery sags, and configuration is persisted to flash so a brownout can't revert it.

### CorAl ([CorAl.java](src/main/java/frc/robot/subsystems/CorAl.java))
Pivot arm + intake rollers (both Kraken X60). The REV Through Bore absolute encoder is the angle reference: the motor's sensor is seeded from it at startup, before each move, and whenever the arm is idle — never mid-move. Motion Magic in degrees, closed-loop holding at targets. Game piece detection uses the CANrange's on-device proximity bit with a 0.3 s rising-edge debounce, and stops the intake automatically.

### Superstructure ([Superstructure.java](src/main/java/frc/robot/Superstructure.java))
The command factory for all coordinated elevator + CorAl motion. The arm (0–160°) sweeps through the elevator's structure, and three contact combinations were measured on the robot: the tucked arm (0°) hits a static elevator part at 10.75" of height; a 5° arm hits the second-stage tube passing 23"; and at full height the arm cannot rotate between 90° and 45° in place.

A **motion planner** builds each move from the real mechanism state at the moment the button is pressed, picking the fastest sequence that stays inside the safe regions:

- **Low-box moves run direct.** Below `LOW_TRAVEL_MAX_HEIGHT` (21") with the arm at/above `ARM_CLEAR_MIN_ANGLE` (5°), nothing contacts — so base→L2, base→L1, and base→algae-low move the arm and elevator together with **no 90° excursion**.
- **Leaving the low box travels at the safe angle (90°)**, but ascents give the elevator a **state-gated head start** (up to the highest height safe for the current arm angle) while the arm swings up, so the swing costs little or no time.
- **Handoff overlaps** — above the low box the arm can't rotate below 90° in place without sweeping into the second-stage tube, so the final rotation overlaps the last part of the climb: past `MID_HANDOFF_HEIGHT` (25", TUNE) for L3 (29", 22.5°) and past `HIGH_HANDOFF_HEIGHT` (40", TUNE) for L4 (52.5", 45°). Leaving those poses mirrors the overlap with the arm swinging up during the initial descent.
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
| **Setup** | Pre/post-match checks | FMS info, battery + CAN health, Alerts, Zero Elevator / Zero CorAl Pivot buttons, sensor and Limelight status, reef camera |
| **Autonomous** | Auto selection & monitoring | Auto chooser, big match timer, **Field widget with live robot pose**, game piece indicator, Alerts |
| **Teleop** | Driving | **Field widget**, match timer, big game-piece box, swerve module widget, elevator/pivot position bars, vision tracking state |
| **Testing** | Diagnostics | Command scheduler, subsystem widgets, motor current/velocity graphs, vision + CANrange readouts |

The robot **switches Elastic to the right tab automatically** on mode changes (disabled → Setup, auto → Autonomous, teleop → Teleop, test → Testing) via ElasticLib ([util/Elastic.java](src/main/java/frc/robot/util/Elastic.java)).

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
2. **Sanity-check the elevator height reading** against a tape measure. The conversion (5.5"/45 per motor rotation) is derived from the real gearing — 45:1 MAXPlanetary → 90° box → 22T #25 sprocket — but confirm the 90° gearbox is 1:1 and the rigging doesn't multiply travel.
3. **Sanity-check the CorAl pivot angle** against the through bore encoder. The 65.41:1 ratio is derived from the real gear train (10:58 → 18:58 → 12:42). Also confirm the through bore is mounted 1:1 on the pivot shaft.
4. **Tune closed-loop gains** (elevator kP/kS/kV/kG via REV Hardware Client, pivot kP/kG via Phoenix Tuner X, `AutoConstants` path-following kP) and test autos.
5. **Tune the vision goal constants**: place the robot flush on the reef base and copy the `Vision/Distance` reading into `REEF_FLUSH_DISTANCE` (same procedure backed up to the coral station for `STATION_FLUSH_DISTANCE`); verify `REEF_BRANCH_OFFSET`, the branch left/right sign, and `LIMELIGHT_FACING_SIGNS` (which cameras face front vs. rear). Verify `MID_HANDOFF_HEIGHT` on the L3 approach **at low speed first**.
6. **Firmware**: 2026 firmware on all CTRE devices (TalonFX, CANcoder, Pigeon 2, CANrange, CANdle), current Spark MAX firmware via the REV Hardware Client, 2026 roboRIO image, Limelight OS 2026.0+.
6a. **Set the CANrange's CAN ID to 62 in Tuner X** (or change `CANRANGE_SENSOR_ID` to its actual ID). It was documented as ID 64, which is **not a legal Phoenix ID (0-62)** — robot code crashed at construction with it (caught by the unit tests).
7. **Set `LEDConstants.LED_COUNT`** to the actual LED strip length.
8. Raise the driver translation scale in RobotContainer (currently 25% for practice) as appropriate.

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
