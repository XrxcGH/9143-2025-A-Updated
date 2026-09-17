package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import java.io.File;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CorAl;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.util.Tunables;

/**
 * RobotContainer owns every subsystem and maps controller inputs to commands.
 * This is the single place to look up "what does this button do".
 *
 * ================================ CONTROLS =================================
 * DRIVER (port 0):
 *   Left stick          - field-centric translation (forward/strafe)
 *   Right stick X       - rotation
 *   A (hold)            - X-lock the wheels (brake)
 *   B (hold)            - point all modules at the left-stick direction
 *   Y (press)           - toggle AprilTag vision tracking (goal follows the
 *                         operator's selected pose: reef flush for L2-L4 on
 *                         the chosen branch, 1 m standoff for L1, flush
 *                         centered for algae, rear-flush at coral stations)
 *   Left trigger        - align on the LEFT reef branch (L2-L4 tracking)
 *   Right trigger       - align on the RIGHT reef branch (L2-L4 tracking)
 *   D-pad               - slow robot-centric nudges (up/down/left/right)
 *   Left bumper         - re-zero field-centric heading
 *   Back/Start + X/Y    - SysId characterization routines (test setup only)
 *
 * OPERATOR (port 1):
 *   All position buttons run coordinated elevator+arm sequences through the
 *   Superstructure, which automatically avoids mechanism contact from any
 *   starting pose.
 *   Left stick Y        - elevator manual control (holds height on release;
 *                         NO collision interlocks - watch the mechanism)
 *   Right stick X       - CorAl pivot manual control (holds angle on release;
 *                         NO collision interlocks - watch the mechanism)
 *   D-pad down          - coral L1 scoring pose (0", 100 deg)
 *   D-pad left          - coral L2 scoring pose (12", 5 deg)
 *   D-pad right         - coral L3 scoring pose (29", 22.5 deg, handoff overlap)
 *   D-pad up            - coral L4 scoring pose (52.5", 45 deg, handoff overlap)
 *   A                   - coral intake (stow to base, rollers until the
 *                         CANrange confirms a game piece)
 *   X                   - eject coral at the current pose (rollers 0.5 s)
 *   B                   - algae low intake (20.5", 160 deg, rollers in)
 *   Back                - algae high intake (37.5", 160 deg, rollers in)
 *   Y                   - algae hold (hold rollers, arm to 90 deg)
 *   Right bumper        - algae score (52.5", 105 deg, then eject)
 *   Left trigger        - stow to base (rollers stopped, arm tucked)
 *   Right trigger       - raise arm to safe travel angle (context-aware)
 *   Left bumper         - reset elevator encoders (DISABLED ONLY, at base)
 *   Start               - reset CorAl pivot encoder (DISABLED ONLY, at base)
 * ===========================================================================
 */
public class RobotContainer {
    /** Top speed from swerve characterization, used to scale driver input. */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Max rotation rate for driver input: 3/4 rotation per second. */
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // ------------------------------------------------------------------
    // Reusable swerve requests for teleop driving (allocated once)
    // ------------------------------------------------------------------
    // NOTE: drive requests use CLOSED-LOOP velocity, not open-loop voltage:
    // every module tracks the true requested ground speed regardless of
    // battery sag, and teleop behavior matches autonomous path following.
    /**
     * Standard field-centric drive. Speed scaling and the matching deadbands
     * are applied per loop in the default command (the scale is a dashboard
     * tunable), so nothing speed-dependent is baked in here.
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    /** X-locks the wheels to resist being pushed. */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    /** Points all modules in a direction without driving (alignment/testing). */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    /** Robot-centric drive used for the slow D-pad nudges. */
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    /** Publishes swerve state to NetworkTables/SignalLogger every odometry update. */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers: driver handles the drivetrain, operator handles mechanisms
    private final CommandXboxController driver_controller = new CommandXboxController(0);
    private final CommandXboxController operator_controller = new CommandXboxController(1);

    // ------------------------------------------------------------------
    // Subsystems
    // ------------------------------------------------------------------
    public final Swerve swerve;
    private final Elevator elevator = new Elevator();
    private final CorAl coral = new CorAl();
    private final LEDs leds;

    @AutoLogOutput (key = "Draggables/DesiredComponents3d")
    public static Pose3d[] desiredComponents3d = {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};
    @AutoLogOutput (key = "Draggables/FreshCode")
    public static boolean freshCode = true;

    /** Coordinated elevator+arm motion with collision interlocks. */
    private final Superstructure superstructure = new Superstructure(elevator, coral);

    /** Dashboard chooser for selecting the autonomous routine (logged through
     *  AdvantageKit so every log records which auto was selected). */
    private final LoggedDashboardChooser<Command> autoChooser;

    /** Central Elastic dashboard publisher; updated from Robot.robotPeriodic(). */
    private final Dashboard dashboard;

    public RobotContainer() {
        // Swerve creates its own Vision instance internally; constructing a
        // second one here would double up pose updates and dashboard widgets.
        swerve = TunerConstants.createDrivetrain();

        // LEDs derive their state from the other subsystems through these
        // suppliers - no further wiring needed anywhere else.
        leds = new LEDs(coral::isGamePieceDetected, swerve::isVisionTrackingEnabled);

        // Auto chooser is populated with every auto in deploy/pathplanner/autos.
        // LoggedDashboardChooser publishes it under SmartDashboard/Auto Mode
        // (Elastic's ComboBox Chooser widget) AND records the selection in
        // the AdvantageKit log.
        autoChooser = new LoggedDashboardChooser<>("Auto Mode", AutoBuilder.buildAutoChooser());

        // Second autonomous option: add every Choreo trajectory from
        // deploy/choreo to the same chooser (see addChoreoAutos).
        addChoreoAutos();

        // Vision alignment goals depend on what the superstructure is doing
        // (L1 standoff vs. flush scoring vs. algae) - wire that in.
        swerve.getVision().setGoalSupplier(superstructure::getGoal);

        // Seed the dashboard-editable tunables (vision distances, handoff
        // heights, ...) with their Constants defaults if not already stored
        // on the roboRIO.
        Tunables.init();

        // All Elastic/NetworkTables publishing is centralized here.
        dashboard = new Dashboard(swerve, elevator, coral, leds, superstructure);

        configureBindings();
    }

    /** Publishes all dashboard data; called every loop by Robot.robotPeriodic(). */
    public void updateDashboard() {
        dashboard.update();
    }

    private void configureBindings() {
        configureSwerveBindings();
        configureSuperstructureBindings();
    }

    // ==================================================================
    // Driver bindings (drivetrain)
    // ==================================================================
    private void configureSwerveBindings() {
        // Default command: field-centric driving from the sticks (closed-loop
        // velocity; see the drive request note above).
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        //
        // Both translation AND rotation are scaled by the "Drive - Teleop
        // Speed Scale" tunable (default 25% for indoor testing; edit it on
        // the dashboard, no redeploy). The deadbands scale with it: a fixed
        // deadband sized for full speed would swallow most of the stick's
        // travel at a small scale.
        swerve.setDefaultCommand(
            swerve.applyRequest(() -> {
                double scale = Tunables.teleopSpeedScale();
                double maxSpeed = MaxSpeed * scale;
                double maxAngularRate = MaxAngularRate * scale;
                return drive
                    .withDeadband(maxSpeed * DriveConstants.STICK_DEADBAND)
                    .withRotationalDeadband(maxAngularRate * DriveConstants.STICK_DEADBAND)
                    .withVelocityX(-driver_controller.getLeftY() * maxSpeed)       // Forward with negative Y (stick up)
                    .withVelocityY(-driver_controller.getLeftX() * maxSpeed)       // Left with negative X
                    .withRotationalRate(-driver_controller.getRightX() * maxAngularRate); // CCW with negative X (stick left)
            })
        );

        // A: X-lock wheels; B: point modules at the left-stick direction
        driver_controller.a().whileTrue(swerve.applyRequest(() -> brake));
        driver_controller.b().whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))
        ));

        // D-pad: slow robot-centric nudges for lining up on field elements
        driver_controller.povUp().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver_controller.povDown().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driver_controller.povLeft().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        driver_controller.povRight().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver_controller.back().and(driver_controller.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driver_controller.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        // Select which reef branch vision tracking centers on for L2-L4
        // (latched; defaults to LEFT on boot)
        driver_controller.leftTrigger().onTrue(Commands.runOnce(() ->
            swerve.getVision().setBranchSide(Vision.BranchSide.LEFT)));
        driver_controller.rightTrigger().onTrue(Commands.runOnce(() ->
            swerve.getVision().setBranchSide(Vision.BranchSide.RIGHT)));

        // Toggle vision tracking on Y, but not while back/start are held
        // (back+Y and start+Y are the SysId test combos above). The toggle
        // keys off whether the tracking command is actually SCHEDULED, not a
        // parallel flag - the command's own finallyDo stops the robot and
        // clears the tracking state whenever it ends, including when another
        // swerve binding (brake, point, nudges, SysId) interrupts it, so the
        // toggle can never desync from reality.
        driver_controller.y()
            .and(driver_controller.back().negate())
            .and(driver_controller.start().negate())
            .onTrue(Commands.runOnce(() -> {
            if (swerve.aprilTagTrackingCommand.isScheduled()) {
                swerve.aprilTagTrackingCommand.cancel();
            } else {
                swerve.setVisionTrackingEnabled(true);
                CommandScheduler.getInstance().schedule(swerve.aprilTagTrackingCommand);
            }
        }));

        // Stream drivetrain state to NetworkTables + SignalLogger for analysis
        swerve.registerTelemetry(logger::telemeterize);
    }

    // ==================================================================
    // Operator bindings (superstructure - coordinated elevator + CorAl)
    // ==================================================================
    // Every position button goes through the Superstructure, which sequences
    // the elevator and arm so they can never reach a contact combination,
    // regardless of where the mechanisms currently are.
    private void configureSuperstructureBindings() {
        // Coral scoring poses on the D-pad (L1 through L4)
        operator_controller.povDown().onTrue(superstructure.goToCoralL1());
        operator_controller.povLeft().onTrue(superstructure.goToCoralL2());
        operator_controller.povRight().onTrue(superstructure.goToCoralL3());
        operator_controller.povUp().onTrue(superstructure.goToCoralL4());

        // Coral handling
        operator_controller.a().onTrue(superstructure.intakeCoral());
        operator_controller.x().onTrue(superstructure.ejectCoral());

        // Algae handling
        operator_controller.b().onTrue(superstructure.intakeAlgaeLow());
        operator_controller.back().onTrue(superstructure.intakeAlgaeHigh());
        operator_controller.y().onTrue(superstructure.holdAlgae());
        operator_controller.rightBumper().onTrue(superstructure.scoreAlgae());

        // Stow (rollers stopped, elevator down, arm tucked) and safe raise
        operator_controller.leftTrigger().onTrue(superstructure.stow());
        operator_controller.rightTrigger().onTrue(superstructure.raiseArm());

        // Encoder resets: ONLY while disabled, with mechanisms at their
        // physical base positions. Zeroing while enabled would shift the
        // reference frame under a latched closed-loop setpoint - the
        // controller would suddenly see a huge error and drive the mechanism
        // hard toward a position that no longer means what it did.
        operator_controller.leftBumper().and(DriverStation::isDisabled)
            .onTrue(Commands.runOnce(() -> elevator.resetEncoders(), elevator).ignoringDisable(true));
        operator_controller.start().and(DriverStation::isDisabled)
            .onTrue(Commands.runOnce(() -> coral.resetPivotEncoder(), coral).ignoringDisable(true));

        // -------- Manual overrides (default commands) --------
        // WARNING: manual control commands the subsystems directly and has
        // NO collision interlocks - the operator must watch the mechanism.

        // Elevator manual control; holds height under closed loop on release
        elevator.setDefaultCommand(Commands.run(() -> {
            double speed = -operator_controller.getLeftY();
            if (Math.abs(speed) > ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
                elevator.manualControl(speed);
            } else if (elevator.isInManualMode()) {
                elevator.holdCurrentPosition();
            }
        }, elevator));

        // Pivot manual control; the subsystem applies the deadband and speed
        // limit and holds the current angle when the stick is released
        coral.setDefaultCommand(Commands.run(() ->
            coral.manualPivotControl(operator_controller.getRightX()), coral));
    }

    /** Returns the autonomous routine selected on the dashboard. */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // ==================================================================
    // Choreo autonomous option (alongside PathPlanner)
    // ==================================================================
    /**
     * Discovers every Choreo trajectory (.traj) in deploy/choreo and adds it
     * to the auto chooser as "Choreo: &lt;name&gt;", so the driver can pick a
     * PathPlanner auto OR a Choreo trajectory from the same dropdown. Draw a
     * trajectory in the Choreo app (saving into src/main/deploy/choreo) and it
     * appears here automatically - exactly how PathPlanner autos are picked up
     * from deploy/pathplanner/autos.
     *
     * The trajectory is followed by the SAME PathPlanner AutoBuilder holonomic
     * controller and AutoConstants gains as the PathPlanner autos: PathPlanner
     * 2026 natively loads Choreo .traj files ({@code fromChoreoTrajectory}),
     * and ChoreoLib has no 2026 release, so this keeps a single, already-tuned
     * path-following code path. A malformed or unreadable file is reported and
     * skipped rather than crashing robot construction.
     */
    private void addChoreoAutos() {
        File choreoDir = new File(Filesystem.getDeployDirectory(), "choreo");
        File[] trajFiles = choreoDir.listFiles((dir, name) -> name.endsWith(".traj"));
        if (trajFiles == null) {
            return; // no deploy/choreo directory present
        }
        Arrays.sort(trajFiles);
        for (File file : trajFiles) {
            String name = file.getName().substring(0, file.getName().length() - ".traj".length());
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
                autoChooser.addOption("Choreo: " + name, choreoAutoCommand(path));
            } catch (Exception ex) {
                DriverStation.reportError("Failed to load Choreo trajectory '" + name + "'", ex.getStackTrace());
            }
        }
    }

    /**
     * Builds a standalone auto from a Choreo-sourced path: reset odometry to
     * the trajectory's starting pose, then follow it. The start pose is
     * alliance-flipped to match how AutoBuilder mirrors the path itself on the
     * red alliance, so a Choreo auto lines up correctly on both alliances.
     */
    private Command choreoAutoCommand(PathPlannerPath path) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                Pose2d start = path.getStartingHolonomicPose().orElse(swerve.getState().Pose);
                swerve.resetPose(AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(start) : start);
            }),
            AutoBuilder.followPath(path)
        );
    }

    /** Called from Robot.disabledInit(): stop every mechanism output. */
    public void disabledInit() {
        elevator.stopElevator();
        coral.stopPivot();
        coral.stopIntake();
    }
}
