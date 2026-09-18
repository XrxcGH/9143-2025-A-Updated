package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CorAl;
import frc.robot.subsystems.Elevator;
// CANdle disabled (no CANdle on the robot): import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Elastic;
import frc.robot.util.Tunables;

/**
 * Central dashboard manager - the ONLY place in the robot code that publishes
 * data for the Elastic dashboard.
 *
 * How the Elastic integration works (Elastic dropped Shuffleboard API
 * support, so this project uses the modern approach):
 *
 *  1. This class publishes plain NetworkTables data (numbers, booleans,
 *     sendables like Field2d and the auto chooser) under /SmartDashboard.
 *  2. The tab layout lives in deploy/elastic-layout.json (Setup, Autonomous,
 *     Teleop, and Testing tabs). Robot.java serves the deploy directory over
 *     HTTP on port 5800, and Elastic loads the layout with
 *     File -> "Load Layout From Robot" (Ctrl+D).
 *  3. Robot.java calls Elastic.selectTab() on mode changes so the dashboard
 *     automatically shows the right tab for each match phase.
 *  4. Persistent problems surface through WPILib Alerts (Elastic's Alerts
 *     widget); sudden mid-match failures additionally fire an Elastic toast
 *     notification so they are impossible to miss.
 *  5. The Testing tab drives each mechanism INDEPENDENTLY: setpoint sliders
 *     (values flow dashboard -> robot, so update() never overwrites them)
 *     plus Go/Run/Stop command buttons that move one mechanism and leave
 *     the other alone (see Superstructure.test*). Tunable "magic numbers"
 *     are edited there too, through WPILib Preferences (see Tunables).
 *
 * Subsystems expose plain getters and know nothing about the dashboard;
 * update() polls them once per loop from Robot.robotPeriodic().
 */
public class Dashboard {
    private final Swerve swerve;
    private final Elevator elevator;
    private final CorAl coral;
    // CANdle disabled (no CANdle on the robot): private final LEDs leds;
    private final Superstructure superstructure;


    // Dashboard-editable test setpoints (Testing tab). These are READ by the
    // robot, never written in update() - a periodic put would clobber the
    // operator's slider the instant they moved it.
    private final NetworkTableEntry testElevatorSetpoint;
    private final NetworkTableEntry testPivotSetpoint;
    private final NetworkTableEntry testIntakeSpeed;

    /** Field widget data: robot pose (and any objects added later, e.g. trajectories). */
    private final Field2d field = new Field2d();

    /** Raw NT table backing Elastic's SwerveDrive widget (needs a ".type" marker). */
    private final NetworkTable swerveWidgetTable;

    // Cached NT entries for the SwerveDrive widget (order: FL angle, FL vel,
    // FR angle, FR vel, BL angle, BL vel, BR angle, BR vel, robot angle) -
    // resolving string-keyed entries on every 20 ms loop is pure waste.
    private final NetworkTableEntry[] swerveWidgetEntries;

    // Precomputed "Vision/<name> Has Target" keys (avoids per-loop concatenation)
    private final String[] visionHasTargetKeys;
    // Field2d objects showing each camera's last fused pose (cleared when stale)
    private final FieldObject2d[] visionFieldObjects;

    // ------------------------------------------------------------------
    // Superstructure visualization
    // ------------------------------------------------------------------
    // Mechanism2d: side-on schematic of the elevator + arm, rendered by
    // Glass and AdvantageScope (both live over NT). Drawing conventions:
    // the elevator ligament points straight up; the arm ligament's angle is
    // relative to the elevator, drawn so 0 deg (tucked) points down along
    // the elevator, 90 deg (safe) points out horizontally.
    private final Mechanism2d superstructureMech = new Mechanism2d(1.5, 2.5);
    private final MechanismLigament2d elevatorLigament;
    private final MechanismLigament2d armLigament;

    // Arm length for both visualizations (meters): pivot axis (through-bore
    // centerline) to the far intake roller axes, 13.9 in in the CAD
    // (9143-2025-A-0000 Leviathan STEP).
    private static final double ARM_LENGTH = 0.352;

    // 3D component-pose model for AdvantageScope's 3D field view: attach a
    // glTF CAD model (File > Import CAD or the online converter) and map
    // these array entries to its articulated components in the 3D config.
    // Robot-relative coordinate frame: X forward, Y left, Z up, origin at
    // the robot center on the floor. All offsets below are PLACEHOLDERS
    // measured as zero - VERIFY against the CAD model's component origins
    // (AdvantageScope docs: "Custom Assets > Articulated components").
    // From the CAD (9143-2025-A-0000 Leviathan STEP, carriage on its hard
    // stop, floor at the wheel contact): the pivot axis (through-bore bore
    // centerline) is 12.01 in forward of the frame center and 13.875 in
    // above the floor.
    private static final double ELEVATOR_X_OFFSET = 0.305;  // Meters forward of robot center
    private static final double ARM_PIVOT_HEIGHT = 0.352;   // Pivot height above the floor at the elevator hard stop (meters)
    @AutoLogOutput (key = "Draggables/Components3d")
    private final Pose3d[] componentPoses = {new Pose3d(), new Pose3d(), new Pose3d()};

    // ------------------------------------------------------------------
    // Persistent alerts - shown in the Alerts widget on every tab that has
    // one, and logged automatically. set(true) shows, set(false) clears.
    // ------------------------------------------------------------------
    private final Alert throughBoreAlert = new Alert(
        "CorAl through bore encoder disconnected - pivot is running on the motor encoder only.",
        AlertType.kError);
    private final Alert elevatorSyncAlert = new Alert(
        "Elevator sides out of sync - check for belt slippage or mechanical binding.",
        AlertType.kWarning);
    private final Alert coralFeedbackAlert = new Alert(
        "CorAl motor encoder disagrees with the through bore - it will re-sync when the arm is idle.",
        AlertType.kWarning);
    private final Alert lowBatteryAlert = new Alert(
        "Battery resting voltage is low - swap the battery before the next match.",
        AlertType.kWarning);
    private final Alert elevatorBelowZeroAlert = new Alert(
        "Elevator reads below its hard-stop height - it was zeroed with the carriage raised. Lower "
            + "it to the hard stop and press Zero Elevator, or every height will land high.",
        AlertType.kWarning);
    private final Alert presetAuditAlert = new Alert("", AlertType.kInfo);
    private final Alert elevatorRatioPendingAlert = new Alert(
        "New elevator calibration (travel ratio / hard-stop height) not applied yet - lower the "
            + "carriage to its hard stop and disable; it applies and re-zeros there automatically.",
        AlertType.kInfo);

    /** Tracks the through bore state so a disconnect fires one toast, not a stream. */
    private boolean throughBoreWasConnected = true;

    /**
     * Registers all sendables and camera streams. Call once from
     * RobotContainer after the subsystems exist. (The auto chooser is a
     * LoggedDashboardChooser that publishes itself - see RobotContainer.)
     */
    public Dashboard(Swerve swerve, Elevator elevator, CorAl coral, /* CANdle disabled: LEDs leds, */
            Superstructure superstructure) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        // CANdle disabled (no CANdle on the robot): this.leds = leds;
        this.superstructure = superstructure;

        // Staged-sequence stations (CAD-derived constants; fixed at compile
        // time) and a one-time audit of the presets against the CAD free
        // corridors - published once, they never change at runtime.
        SmartDashboard.putString("Superstructure/Handoffs", Superstructure.stationSummary());
        String audit = Superstructure.presetAuditMessage();
        presetAuditAlert.setText(audit.isEmpty() ? "" : audit);
        presetAuditAlert.set(!audit.isEmpty());

        // --- Independent mechanism test controls (Testing tab) ---
        // Setpoints are plain NT doubles the dashboard sliders write to;
        // setDefaultDouble only seeds a value if none exists yet.
        NetworkTable smartDashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        testElevatorSetpoint = smartDashboard.getEntry("Testing/Elevator Setpoint (in)");
        testElevatorSetpoint.setDefaultDouble(0.0);
        testPivotSetpoint = smartDashboard.getEntry("Testing/Pivot Setpoint (deg)");
        testPivotSetpoint.setDefaultDouble(0.0);
        testIntakeSpeed = smartDashboard.getEntry("Testing/Intake Speed");
        testIntakeSpeed.setDefaultDouble(0.1);

        // Go/Run/Stop buttons: each moves exactly one mechanism (the
        // Superstructure refuses - with a toast - any single-mechanism move
        // the collision model says is unsafe from the CURRENT pose).
        SmartDashboard.putData("Testing/Elevator Go",
            superstructure.testElevatorTo(() -> testElevatorSetpoint.getDouble(0.0))
                .withName("Elevator Go"));
        SmartDashboard.putData("Testing/Pivot Go",
            superstructure.testPivotTo(() -> testPivotSetpoint.getDouble(0.0))
                .withName("Pivot Go"));
        SmartDashboard.putData("Testing/Intake Run",
            superstructure.testIntakeRun(() -> testIntakeSpeed.getDouble(0.0))
                .withName("Intake Run"));
        SmartDashboard.putData("Testing/Intake Stop",
            superstructure.testIntakeStop().withName("Intake Stop"));

        // Restores every dashboard-tunable value (Robot Preferences widget)
        // to its Constants default; usable while disabled.
        SmartDashboard.putData("Testing/Reset Tunables",
            Commands.runOnce(Tunables::resetToDefaults)
                .ignoringDisable(true).withName("Reset Tunables"));

        // --- Superstructure Mechanism2d (Glass / AdvantageScope) ---
        elevatorLigament = superstructureMech.getRoot("Superstructure", 0.75, 0.05)
            .append(new MechanismLigament2d("Elevator", ARM_PIVOT_HEIGHT, 90, 8,
                new Color8Bit(Color.kOrange)));
        armLigament = elevatorLigament
            .append(new MechanismLigament2d("Arm", ARM_LENGTH, -180, 6,
                new Color8Bit(Color.kCyan)));

        // --- Sendables (registered once; NT keeps them updated) ---
        // Field widget: realtime robot location on the field drawing
        SmartDashboard.putData("Field", field);
        // Elevator + arm schematic (viewable in Glass and AdvantageScope)
        SmartDashboard.putData("Superstructure Mechanism", superstructureMech);
        // Command scheduler view for the Testing tab
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        // Subsystem widgets (show default/current command) for the Testing tab
        SmartDashboard.putData("Elevator Subsystem", elevator);
        SmartDashboard.putData("CorAl Subsystem", coral);

        // --- Pre-match utility buttons (Command widgets on the Setup tab) ---
        // ignoringDisable lets the pit crew zero mechanisms without enabling -
        // and ONLY without enabling: zeroing under a latched closed-loop
        // setpoint shifts its frame and drives the mechanism hard. (These ran
        // while enabled too; the controller bindings never did.)
        SmartDashboard.putData("Zero Elevator",
            Commands.runOnce(elevator::resetEncoders, elevator)
                .onlyIf(edu.wpi.first.wpilibj.DriverStation::isDisabled)
                .ignoringDisable(true).withName("Zero Elevator"));
        SmartDashboard.putData("Zero CorAl Pivot",
            Commands.runOnce(coral::resetPivotEncoder, coral)
                .onlyIf(edu.wpi.first.wpilibj.DriverStation::isDisabled)
                .ignoringDisable(true).withName("Zero CorAl Pivot"));

        // --- Elastic SwerveDrive widget ---
        // Elastic identifies the widget by a ".type" marker and reads the
        // module entries published in update() (angles in radians, m/s).
        swerveWidgetTable = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getSubTable("Swerve Drive");
        swerveWidgetTable.getEntry(".type").setString("SwerveDrive");
        swerveWidgetEntries = new NetworkTableEntry[] {
            swerveWidgetTable.getEntry("Front Left Angle"),
            swerveWidgetTable.getEntry("Front Left Velocity"),
            swerveWidgetTable.getEntry("Front Right Angle"),
            swerveWidgetTable.getEntry("Front Right Velocity"),
            swerveWidgetTable.getEntry("Back Left Angle"),
            swerveWidgetTable.getEntry("Back Left Velocity"),
            swerveWidgetTable.getEntry("Back Right Angle"),
            swerveWidgetTable.getEntry("Back Right Velocity"),
            swerveWidgetTable.getEntry("Robot Angle"),
        };

        visionHasTargetKeys = new String[VisionConstants.LIMELIGHT_NAMES.length];
        visionFieldObjects = new FieldObject2d[VisionConstants.LIMELIGHT_NAMES.length];
        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            visionHasTargetKeys[i] = "Vision/" + VisionConstants.LIMELIGHT_NAMES[i] + " Has Target";
            visionFieldObjects[i] = field.getObject("Vision " + VisionConstants.LIMELIGHT_NAMES[i]);
        }

        // --- Limelight camera streams ---
        // Registers each Limelight's MJPEG stream under /CameraPublisher so
        // Elastic's Camera Stream widget can display it. The dashboard pulls
        // video straight from the camera; nothing streams through the roboRIO.
        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            CameraServer.addCamera(new HttpCamera(
                "limelight-" + name,
                "http://limelight-" + name + ".local:5800/stream.mjpg",
                HttpCameraKind.kMJPGStreamer));
        }
    }

    /**
     * Publishes all live values. Called every loop from Robot.robotPeriodic().
     */
    public void update() {
        // --- Field + drivetrain ---
        // A private snapshot: getState() returns the object the odometry
        // thread rewrites at 100 Hz, so pose/speeds/modules read below would
        // otherwise come from different odometry ticks
        var driveState = swerve.getStateCopy();
        field.setRobotPose(driveState.Pose);

        SmartDashboard.putNumber("Swerve/Speed",
            Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond));
        SmartDashboard.putNumber("Swerve/Heading", driveState.Pose.getRotation().getDegrees());
        SmartDashboard.putBoolean("Swerve/Vision Tracking", swerve.isVisionTrackingEnabled());

        // SwerveDrive widget entries (module order: FL, FR, BL, BR)
        if (driveState.ModuleStates != null && driveState.ModuleStates.length == 4) {
            for (int i = 0; i < 4; i++) {
                swerveWidgetEntries[i * 2].setDouble(driveState.ModuleStates[i].angle.getRadians());
                swerveWidgetEntries[i * 2 + 1].setDouble(driveState.ModuleStates[i].speedMetersPerSecond);
            }
            swerveWidgetEntries[8].setDouble(driveState.Pose.getRotation().getRadians());
        }

        // --- Superstructure visualization ---
        double heightMeters = Units.inchesToMeters(elevator.getCurrentPosition());
        double armAngleDeg = coral.getPivotAngle();

        // Mechanism2d: elevator ligament grows with height; arm ligament is
        // drawn relative to the elevator (0 deg tucked = down, 90 = out).
        elevatorLigament.setLength(ARM_PIVOT_HEIGHT + heightMeters);
        armLigament.setAngle(armAngleDeg - 180.0);

        // 3D component poses for AdvantageScope (robot-relative: X forward,
        // Y left, Z up). Cascade rigging: the middle stage rises at half the
        // carriage speed. Arm pitches about the Y axis; the sign/zero must
        // match the CAD component's modeled orientation - VERIFY in
        // AdvantageScope and flip/offset here if the model swings backward.
        componentPoses[LoggingConstants.MIDDLE_STAGE_INDEX] =
            new Pose3d(ELEVATOR_X_OFFSET, 0, heightMeters / 2.0, Rotation3d.kZero);
        componentPoses[LoggingConstants.CARRIAGE_INDEX] =
            new Pose3d(ELEVATOR_X_OFFSET, 0, heightMeters, Rotation3d.kZero);
        componentPoses[LoggingConstants.ARM_INDEX] =
            new Pose3d(ELEVATOR_X_OFFSET, 0, ARM_PIVOT_HEIGHT + heightMeters,
                new Rotation3d(0, -Units.degreesToRadians(armAngleDeg), 0));

        // --- AdvantageKit structured outputs (.wpilog + RLOG live stream) ---
        // These are the review-critical fields for AdvantageScope: 2D/3D
        // field views, swerve visualization, and mechanism traces.
        Logger.recordOutput("RobotState/Pose", Pose2d.struct, driveState.Pose);
        Logger.recordOutput("RobotState/Speeds", ChassisSpeeds.struct, driveState.Speeds);
        if (driveState.ModuleStates != null && driveState.ModuleStates.length == 4) {
            Logger.recordOutput("RobotState/ModuleStates", SwerveModuleState.struct, driveState.ModuleStates);
            Logger.recordOutput("RobotState/ModuleTargets", SwerveModuleState.struct, driveState.ModuleTargets);
        }
        Logger.recordOutput("RobotState/ComponentPoses", Pose3d.struct, componentPoses);
        Logger.recordOutput("Elevator/HeightInches", elevator.getCurrentPosition());
        Logger.recordOutput("Elevator/TargetInches", elevator.getTargetPosition());
        Logger.recordOutput("CorAl/AngleDegrees", armAngleDeg);
        Logger.recordOutput("CorAl/TargetDegrees", coral.getTargetAngle());
        Logger.recordOutput("CorAl/GamePiece", coral.isGamePieceDetected());
        // (Vision/BestTag is logged in the Vision block below, from the same
        // single best-target read as the dashboard values.)

        // --- Match / robot vitals ---
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("CAN Utilization",
            RobotController.getCANStatus().percentBusUtilization);

        // --- Elevator ---
        SmartDashboard.putNumber("Elevator/Height", elevator.getCurrentPosition());
        SmartDashboard.putNumber("Elevator/Target", elevator.getTargetPosition());
        SmartDashboard.putNumber("Elevator/Velocity", elevator.getVelocity());
        SmartDashboard.putBoolean("Elevator/At Target", elevator.isAtTargetPosition());
        SmartDashboard.putBoolean("Elevator/Manual Mode", elevator.isInManualMode());
        SmartDashboard.putNumber("Elevator/Left Current", elevator.getLeftCurrent());
        SmartDashboard.putNumber("Elevator/Right Current", elevator.getRightCurrent());
        SmartDashboard.putNumber("Elevator/Left Output", elevator.getLeftOutput());
        // Voltage the leader is applying (output is voltage-compensated to
        // 12 V) MINUS kS. The Spark MAX adds +kS whenever the profile is at
        // rest (REVLib 2026 - measured in REV's own sim: it does not follow
        // the sign of the error), so the raw output holding still is
        // kG + kS + kP x error. With kS taken off, what is left is what kG
        // should be - "set kG to Hold Volts" used to over-set it by kS.
        SmartDashboard.putNumber("Elevator/Hold Volts",
            elevator.getLeftOutput() * ElevatorConstants.ELEVATOR_NOMINAL_VOLTAGE - elevator.staticFeedforward());
        SmartDashboard.putNumber("Elevator/Right Output", elevator.getRightOutput());
        // Calibration in effect on the controllers (the tunables may still be pending)
        SmartDashboard.putNumber("Elevator/Travel Ratio", elevator.travelRatio());
        SmartDashboard.putNumber("Elevator/Height At Hard Stop", elevator.zeroHeight());
        // Follower encoder, raw sign: expected to track Height in magnitude
        // (and read NEGATIVE while raised, since the follower spins opposite)
        SmartDashboard.putNumber("Elevator/Follower Height", elevator.getFollowerPosition());

        // --- CorAl ---
        SmartDashboard.putNumber("CorAl/Angle", coral.getPivotAngle());
        SmartDashboard.putNumber("CorAl/Target", coral.getTargetAngle());
        SmartDashboard.putNumber("CorAl/Motor Angle", coral.getMotorAngle());
        SmartDashboard.putBoolean("CorAl/At Target", coral.isAtTargetAngle());
        SmartDashboard.putNumber("CorAl/Pivot Velocity", coral.getPivotVelocity());
        SmartDashboard.putBoolean("CorAl/Game Piece", coral.isGamePieceDetected());
        SmartDashboard.putBoolean("CorAl/Through Bore OK", coral.isThroughBoreConnected());
        SmartDashboard.putNumber("CorAl/CANrange Distance", coral.getCANRangeDistance());
        // Detection tuning readouts: set "CorAl - Coral Detect Distance"
        // halfway between the empty-claw and coral distances; Raw Detect
        // is the sensor's verdict before the debounce.
        SmartDashboard.putNumber("CorAl/CANrange Signal Strength", coral.getCANRangeSignalStrength());
        SmartDashboard.putString("CorAl/CANrange Health", coral.getCANRangeHealth());
        SmartDashboard.putBoolean("CorAl/CANrange Raw Detect", coral.isCANRangeRawDetected());
        SmartDashboard.putNumber("CorAl/CANrange Threshold", coral.getDetectThreshold());
        SmartDashboard.putNumber("CorAl/CANrange Min Signal Strength", coral.getMinSignalStrength());
        SmartDashboard.putString("CorAl/CANrange Detect When",
            coral.isDetectWhenCloser() ? "closer than threshold" : "farther than threshold");
        SmartDashboard.putBoolean("CorAl/CANrange Detection Enabled", coral.isDetectionEnabled());
        SmartDashboard.putNumber("CorAl/Pivot Current", coral.getPivotCurrent());
        SmartDashboard.putNumber("CorAl/Intake Current", coral.getIntakeCurrent());
        SmartDashboard.putNumber("CorAl/Pivot Output", coral.getPivotOutput());
        SmartDashboard.putNumber("CorAl/Intake Output", coral.getIntakeOutput());

        // --- Vision ---
        // Best VISIBLE tag (any trackable class, ignoring the goal filter and
        // the latch) so the readouts work with the robot pushed into position
        // while disabled; positions are in the ROBOT frame
        var vision = swerve.getVision();
        var bestTarget = vision.getBestVisibleTarget();
        SmartDashboard.putNumber("Vision/Best Tag",
            bestTarget.map(t -> (double) t.id).orElse(-1.0));
        SmartDashboard.putNumber("Vision/TX",
            bestTarget.map(t -> t.tx).orElse(0.0));
        // Forward distance from the robot center to the tag (negative = behind)
        SmartDashboard.putNumber("Vision/Distance",
            bestTarget.map(t -> t.robotFrame.getX()).orElse(0.0));
        // Lateral position of the tag (+ = to the robot's left)
        SmartDashboard.putNumber("Vision/Lateral",
            bestTarget.map(t -> t.robotFrame.getY()).orElse(0.0));
        SmartDashboard.putNumber("Vision/Square Heading",
            bestTarget.flatMap(t -> t.squareHeading).map(Rotation2d::getDegrees).orElse(0.0));
        Logger.recordOutput("Vision/BestTag", bestTarget.map(t -> t.id).orElse(-1));
        for (int i = 0; i < visionHasTargetKeys.length; i++) {
            SmartDashboard.putBoolean(visionHasTargetKeys[i], vision.hasTarget(i));
            // Last pose fused from each camera, drawn on the field beside the
            // robot; cleared once that camera has not fused for a second
            final int camera = i;
            vision.getLastFusedPose(i).ifPresentOrElse(
                visionFieldObjects[camera]::setPose,
                () -> visionFieldObjects[camera].setPoses(java.util.List.of()));
        }
        SmartDashboard.putString("Vision/Branch Side", vision.getBranchSide().name());
        SmartDashboard.putNumber("Vision/Latched Tag", vision.getLatchedTagId());
        SmartDashboard.putNumber("Vision/Heading Offset", vision.getLatchedHeadingOffsetDegrees());
        SmartDashboard.putBoolean("Vision/Heading Seed Fresh", vision.hasFreshHeadingSeed());
        SmartDashboard.putBoolean("Vision/Reseeding Heading", vision.isReseedingHeading());
        SmartDashboard.putBoolean("Vision/Auto Kept Heading", swerve.lastAutoResetKeptHeading());
        // Alignment servo state (robot frame)
        SmartDashboard.putBoolean("Vision/Target Visible", swerve.isAlignmentTargetVisible());
        SmartDashboard.putBoolean("Vision/Target From Memory",
            vision.getBestTarget().map(t -> t.fromMemory).orElse(false));
        SmartDashboard.putBoolean("Vision/Aligned", swerve.isAligned());
        SmartDashboard.putNumber("Vision/Forward Error", swerve.getAlignmentForwardError());
        SmartDashboard.putNumber("Vision/Lateral Error", swerve.getAlignmentLateralError());
        SmartDashboard.putNumber("Vision/Heading Error", swerve.getAlignmentHeadingErrorDegrees());

        // --- LEDs (CANdle code commented out - no CANdle on the robot) ---
        // Restore with the subsystem:
        // SmartDashboard.putString("LEDs/State",
        //     leds.getState() != null ? leds.getState().name() : "INIT");
        SmartDashboard.putString("LEDs/State", "CANdle code commented out");


        // --- Alerts (persistent conditions) ---
        boolean throughBoreConnected = coral.isThroughBoreConnected();
        throughBoreAlert.set(!throughBoreConnected);
        elevatorSyncAlert.set(!elevator.sidesInSync());
        elevatorBelowZeroAlert.set(elevator.readsBelowZero());
        elevatorRatioPendingAlert.set(elevator.isTravelRatioChangePending());
        coralFeedbackAlert.set(throughBoreConnected && !coral.isMotorFeedbackValid());
        // Resting-voltage check only while disabled - voltage sags under
        // load during a match are normal and would nag the drive team.
        lowBatteryAlert.set(DriverStation.isDisabled()
            && RobotController.getBatteryVoltage() < 12.0);

        // --- One-shot toast when the through bore drops out ---
        if (throughBoreWasConnected && !throughBoreConnected) {
            Elastic.sendNotification(new Elastic.Notification(
                Elastic.NotificationLevel.ERROR,
                "Through Bore Disconnected",
                "CorAl pivot angle is now motor-encoder only. Avoid re-zeroing until fixed."));
        }
        throughBoreWasConnected = throughBoreConnected;
    }
}
