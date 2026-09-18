package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import java.io.File;
import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CorAl;
// CANdle disabled (no CANdle on the robot): import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.util.Rumble;
import frc.robot.util.Tunables;

/**
 * RobotContainer owns every subsystem and maps controller inputs to commands.
 * This is the single place to look up "what does this button do".
 *
 * ================================ CONTROLS =================================
 * One coral cycle is LT -> a face button -> RT: index fingers and the right
 * thumb. A pose button first; RT always releases the piece (coral or algae).
 *
 * DRIVER (port 0):
 *   Left stick          - field-centric translation (scaled down automatically
 *                         as the carriage rises: 100 % below 16.5 in, 40 % at 45 in+)
 *   Right stick X       - rotation
 *   Left / right trigger (HOLD) - align on the left / right reef branch; either
 *                         trigger centers on the coral station when stowed
 *                         and empty, and on the barge / processor with an
 *                         algae carried or at the barge pose (those two need
 *                         the front camera's pose measured first - see
 *                         VisionConstants.LIMELIGHT_POSES); release = sticks
 *   Right bumper        - SCORE (same gated command as operator RT; while
 *                         aligning it also waits for "aligned")
 *   Left bumper         - driver heading zero: the way the robot faces now =
 *                         stick forward (back+LB also re-seeds the pose heading)
 *   A (hold)            - X-lock the wheels (brake)
 *   Y                   - re-seed the pose heading from the AprilTags in view
 *                         (MegaTag1 fused for 2 s); the driver's own "forward"
 *                         does not move
 *   D-pad               - slow robot-centric nudges, all 8 directions
 *   B, Back/Start + X/Y - point wheels / SysId: TEST MODE ONLY
 *
 * OPERATOR (port 1):
 *   A / X / B / Y       - coral L1 / L2 / L3 / L4 (a diamond under the right
 *                         thumb: bottom / left / right / top)
 *   Right trigger       - SCORE: waits until the measured pose is reached, runs
 *                         the rollers until the coral has left, then goes home
 *                         by itself (after L3/L4 only once the robot has backed
 *                         0.35 m away - the exit swings the claw past the bumper)
 *   Left trigger        - HOME: stow + intake until a coral is detected; with an
 *                         algae held: carry it low at the travel angle
 *   Left bumper (HOLD)  - MANUAL take-over: cancels the running move and stops
 *                         both mechanisms; while held left stick Y = elevator,
 *                         right stick Y = pivot (NO collision interlocks), RT /
 *                         LT = rollers out / in; release = hold position.
 *                         The sticks do nothing unless LB is held.
 *   Right bumper        - barge pose (RT then fires the algae)
 *   D-pad up / down     - algae high / low intake (diagonals count)
 *   D-pad left          - algae hold in place (hold rollers, arm to 100 deg)
 *   D-pad right         - raise the arm to the safe travel angle, in place
 *   Back / Start, held 1 s, DISABLED only - zero the elevator / the pivot
 *                         (mechanism at its base)
 *
 * RUMBLE: coral acquired (both, one long) - pose reached (operator, two short)
 *   - aligned and pose reached (driver, steady) - scored at L3/L4, back away
 *   (driver, slow pulse) - RT with nothing to score from (operator, one tick).
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
    // Drive requests use closed-loop velocity, not open-loop voltage:
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
    /** Haptic cues (coral acquired, pose reached, aligned, back away). */
    private final Rumble driverRumble = new Rumble(driver_controller);
    private final Rumble operatorRumble = new Rumble(operator_controller);

    // ------------------------------------------------------------------
    // Subsystems
    // ------------------------------------------------------------------
    public final Swerve swerve;
    private final Elevator elevator = new Elevator();
    private final CorAl coral = new CorAl();
    // CANdle disabled (no CANdle on the robot): private final LEDs leds;

    /** Coordinated elevator+arm motion with collision interlocks. */
    private final Superstructure superstructure = new Superstructure(elevator, coral);

    /** Dashboard chooser for selecting the autonomous routine (logged through
     *  AdvantageKit so every log records which auto was selected). */
    private final LoggedDashboardChooser<Command> autoChooser;
    /** Raised when PathPlanner could not be configured (no autos will be offered). */
    private final Alert autoBuilderAlert = new Alert(
        "PathPlanner AutoBuilder not configured (deploy/pathplanner/settings.json missing or invalid): no autos available",
        Alert.AlertType.kError);

    /** Central Elastic dashboard publisher; updated from Robot.robotPeriodic(). */
    private final Dashboard dashboard;

    public RobotContainer() {
        // Swerve creates its own Vision instance internally; constructing a
        // second one here would double up pose updates and dashboard widgets.
        swerve = TunerConstants.createDrivetrain();

        // CANdle disabled (no CANdle on the robot). LEDs derive their state
        // from the other subsystems through these suppliers; restore this
        // line, the field, the import, and the Dashboard argument together:
        // leds = new LEDs(coral::isGamePieceDetected, swerve::isVisionTrackingEnabled);

        // Named commands for the PathPlanner autos (registered before the
        // autos are loaded by buildAutoChooser). "score*" = raise, eject,
        // stow, so the robot drives away with the mechanism tucked;
        // "intakeCoral" = stow, then run the rollers until the CANrange
        // confirms a coral, with the timeout on the roller wait only (the
        // stow's own settle timeout is as long, so a timeout around both
        // could expire before the rollers start) so an empty station cannot
        // stall the routine. PathPlanner wraps each use, so one registration
        // may appear several times in an auto.
        NamedCommands.registerCommand("scoreL4",
            superstructure.goToCoralL4().andThen(superstructure.ejectCoral()).andThen(superstructure.stow()));
        NamedCommands.registerCommand("scoreL3",
            superstructure.goToCoralL3().andThen(superstructure.ejectCoral()).andThen(superstructure.stow()));
        NamedCommands.registerCommand("scoreL2",
            superstructure.goToCoralL2().andThen(superstructure.ejectCoral()).andThen(superstructure.stow()));
        NamedCommands.registerCommand("intakeCoral",
            superstructure.stow().andThen(
                superstructure.intakeRollers().withTimeout(Constants.AutoConstants.AUTO_INTAKE_TIMEOUT_SECONDS)));
        NamedCommands.registerCommand("stow", superstructure.stow());
        // Building blocks for autos that overlap the mechanism with driving
        // instead of doing everything in place at the reef: "prepL*" as an
        // event marker on the approach path (the pose is reached as the
        // robot arrives), "ejectCoral" at the reef, and "stowAfterBackingOff"
        // at the start of the departing path - it waits until the robot has
        // moved 0.35 m before the arm swings out, because leaving L3 / L4
        // puts the claw 9-12 in past the front bumper. The "score*" commands
        // above still do all three in place.
        NamedCommands.registerCommand("prepL4", superstructure.goToCoralL4());
        NamedCommands.registerCommand("prepL3", superstructure.goToCoralL3());
        NamedCommands.registerCommand("prepL2", superstructure.goToCoralL2());
        NamedCommands.registerCommand("ejectCoral", superstructure.ejectCoral());
        NamedCommands.registerCommand("stowAfterBackingOff",
            superstructure.stowAfterBackingOff(() -> swerve.getStateCopy().Pose));

        // The auto chooser is populated with every auto in
        // deploy/pathplanner/autos. LoggedDashboardChooser publishes it under
        // SmartDashboard/Auto Mode (Elastic's ComboBox Chooser widget) and
        // records the selection in the AdvantageKit log.
        SendableChooser<Command> chooser;
        if (AutoBuilder.isConfigured()) {
            chooser = AutoBuilder.buildAutoChooser();
        } else {
            // Swerve.configureAutoBuilder already reported why. buildAutoChooser
            // would throw here and take the whole robot program down with it.
            chooser = new SendableChooser<>();
            chooser.setDefaultOption("None (AutoBuilder not configured)", Commands.none());
            autoBuilderAlert.set(true);
        }
        autoChooser = new LoggedDashboardChooser<>("Auto Mode", chooser);

        // Second autonomous option: add every Choreo trajectory from
        // deploy/choreo to the same chooser (see addChoreoAutos).
        addChoreoAutos();

        // Vision alignment goals depend on what the superstructure is doing
        // (L1 standoff vs. flush scoring vs. algae); the suppliers below wire
        // that in.
        // Reef safety (see Superstructure.setNearReefSupplier and
        // Vision.getTrackingGoal): moves into / out of L3 and L4 wait while a
        // reef tag is right in front of the bumper, and the L3 / L4 alignment
        // holds a standoff back until the pose is reached and backs out to it
        // after the score. (The Superstructure holds "near" for 0.4 s after
        // the last sighting, so one dropped camera frame does not release a
        // swing at the reef.)
        superstructure.setNearReefSupplier(() -> swerve.getVision().getBestVisibleTarget()
            .map(t -> t.tagClass == Constants.VisionConstants.TagClass.REEF
                && t.robotFrame.getX() < Constants.VisionConstants.NEAR_REEF_DISTANCE
                && Math.abs(t.robotFrame.getY()) < Constants.VisionConstants.NEAR_REEF_LATERAL)
            .orElse(false));
        swerve.getVision().setReefStandoffSuppliers(
            superstructure::readyToScore, superstructure::isWaitingForBackOff);

        // With a coral in the claw but no level pressed yet (goal still STOW)
        // the next stop is the reef, not a station, so the aligner is given a
        // reef goal: a STOW goal accepts only station tags, and the robot
        // would sit still in front of the reef.
        swerve.getVision().setGoalSupplier(() ->
            superstructure.getGoal() == Superstructure.Goal.STOW && coral.isGamePieceDetected()
                ? Superstructure.Goal.CORAL_L4 : superstructure.getGoal());

        // (Tunables.init() runs in Robot before this container is built, so
        // the subsystems above were configured from the stored values.)

        // All Elastic/NetworkTables publishing is centralized here.
        dashboard = new Dashboard(swerve, elevator, coral, /* CANdle disabled: leds, */ superstructure);

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
    // Driver bindings (drivetrain + align + score)
    // ==================================================================

    /** Above this carriage height the driver's speed starts to be scaled down... */
    private static final double SLOW_START_HEIGHT = 16.5;   // in (the low box roof)
    /** ...reaching the minimum scale at this height. */
    private static final double SLOW_FULL_HEIGHT = 45.0;    // in
    private static final double SLOW_MIN_SCALE = 0.40;

    /** 1.0 with the carriage low, falling linearly to SLOW_MIN_SCALE as it rises. */
    private double heightSpeedScale() {
        double t = MathUtil.clamp((elevator.getCurrentPosition() - SLOW_START_HEIGHT)
            / (SLOW_FULL_HEIGHT - SLOW_START_HEIGHT), 0.0, 1.0);
        return MathUtil.interpolate(1.0, SLOW_MIN_SCALE, t);
    }

    /**
     * Hold-to-align: selects the branch, tracks while held, and hands the
     * sticks straight back on release (the tracking command's own finallyDo
     * stops the robot and clears the tracking state). One new command per
     * binding - a command instance cannot sit in two compositions.
     */
    private Command alignTo(Vision.BranchSide side) {
        return Commands.runOnce(() -> {
                swerve.getVision().setBranchSide(side);
                swerve.setVisionTrackingEnabled(true);
            })
            .andThen(swerve.createAprilTagTrackingCommand())
            .withName("Align " + side);
    }

    private void configureSwerveBindings() {
        swerve.setDefaultCommand(
            swerve.applyRequest(() -> {
                double scale = Tunables.teleopSpeedScale() * heightSpeedScale();
                double maxSpeed = MaxSpeed * scale;
                double maxAngularRate = MaxAngularRate * scale;
                return drive
                    .withDeadband(maxSpeed * DriveConstants.STICK_DEADBAND)
                    .withRotationalDeadband(maxAngularRate * DriveConstants.STICK_DEADBAND)
                    .withVelocityX(-driver_controller.getLeftY() * maxSpeed)
                    .withVelocityY(-driver_controller.getLeftX() * maxSpeed)
                    .withRotationalRate(-driver_controller.getRightX() * maxAngularRate);
            })
        );

        Trigger testMode = new Trigger(DriverStation::isTest);

        // A: X-lock. B (point wheels) and the SysId chords only exist in Test mode.
        driver_controller.a().whileTrue(swerve.applyRequest(() -> brake));
        driver_controller.b().and(testMode).whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))));
        driver_controller.back().and(driver_controller.y()).and(testMode).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).and(testMode).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).and(testMode).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).and(testMode).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // D-pad nudges in all eight directions from the POV angle, so a thumb
        // that lands on a diagonal still moves the robot (povUp() and the
        // other cardinal triggers are true only at exactly their own angle,
        // so bindings on those alone would ignore a 45-degree press).
        new Trigger(() -> driver_controller.getHID().getPOV() >= 0).whileTrue(swerve.applyRequest(() -> {
            double pov = Math.toRadians(driver_controller.getHID().getPOV()); // 0 = up, clockwise
            return forwardStraight.withVelocityX(0.5 * Math.cos(pov)).withVelocityY(-0.5 * Math.sin(pov));
        }));

        // Driver heading zero on left bumper: "the way the robot faces now is
        // forward on my stick". It only moves the driver's frame (held in the
        // raw gyro frame - see Swerve.periodic), never the pose estimator's
        // heading, so it is safe at any time. While disabled with no tag
        // supplying a heading it also seeds the pose heading to the
        // alliance's forward direction; back + left bumper forces that seed.
        driver_controller.leftBumper().and(driver_controller.back().negate())
            .onTrue(Commands.runOnce(() -> swerve.zeroDriverHeading(
                DriverStation.isDisabled() && !swerve.getVision().hasFreshHeadingSeed())).ignoringDisable(true));
        driver_controller.back().and(driver_controller.leftBumper())
            .onTrue(Commands.runOnce(() -> swerve.zeroDriverHeading(true)).ignoringDisable(true));

        // Y: correct the pose heading from tag geometry. While enabled only
        // MegaTag2 is fused, and MegaTag2 takes its heading from the pose, so
        // a heading that has drifted (a hard hit, a long match) is never
        // corrected by it; this fuses MegaTag1, whose solve carries its own
        // heading, for HEADING_RESEED_WINDOW_SECONDS. With no tag in view it
        // does nothing. The driver's frame is held in the raw gyro frame, so
        // "forward" on the stick does not move. (Not in Test mode, where
        // Back / Start + Y are the SysId bindings.)
        driver_controller.y().and(testMode.negate())
            .onTrue(Commands.runOnce(() -> swerve.getVision().requestHeadingReseed()).ignoringDisable(true));

        // Hold a trigger to align on that side's branch; release = sticks.
        driver_controller.leftTrigger(0.3).whileTrue(alignTo(Vision.BranchSide.LEFT));
        driver_controller.rightTrigger(0.3).whileTrue(alignTo(Vision.BranchSide.RIGHT));

        swerve.registerTelemetry(logger::telemeterize);
    }

    // ==================================================================
    // Operator bindings (superstructure)
    // ==================================================================
    private void configureSuperstructureBindings() {
        // LB is the manual take-over. Everything automatic is gated on it
        // being up, so a held LB means "sticks only" with no surprises.
        Trigger manual = operator_controller.leftBumper();
        Trigger auto = manual.negate();
        Trigger teleop = new Trigger(DriverStation::isTeleopEnabled);
        Trigger disabled = new Trigger(DriverStation::isDisabled);

        // ---- Coral levels: a diamond under the right thumb ----
        operator_controller.a().and(auto).onTrue(superstructure.goToCoralL1()); // bottom
        operator_controller.x().and(auto).onTrue(superstructure.goToCoralL2()); // left
        operator_controller.b().and(auto).onTrue(superstructure.goToCoralL3()); // right
        operator_controller.y().and(auto).onTrue(superstructure.goToCoralL4()); // top

        // ---- Triggers: game piece in / game piece out ----
        // HOME: stow (+ intake rollers if empty), or algae carry if one is held.
        operator_controller.leftTrigger().and(auto).onTrue(superstructure.home());

        // SCORE: gated on the measured pose. Pulled early it waits -
        // the rising edge of (trigger AND ready) is what fires - so it can
        // never interrupt a staged move that is still on its way to the pose.
        Trigger ready = new Trigger(superstructure::readyToScore);
        Command score = superstructure.score(() -> swerve.getStateCopy().Pose);
        operator_controller.rightTrigger().and(auto).and(ready).onTrue(score);
        // Driver's copy: also waits for the aligner when the aligner is in use.
        driver_controller.rightBumper().and(auto).and(ready) // not while the operator holds the manual take-over
            .and(() -> !swerve.isVisionTrackingEnabled() || swerve.isAligned())
            .onTrue(score);

        // ---- Algae on the left thumb: up = high, down = low, left = hold ----
        // Diagonals count toward up/down so a sloppy press is not lost.
        Trigger dpadUp = operator_controller.povUp()
            .or(operator_controller.povUpLeft()).or(operator_controller.povUpRight());
        Trigger dpadDown = operator_controller.povDown()
            .or(operator_controller.povDownLeft()).or(operator_controller.povDownRight());
        dpadUp.and(auto).onTrue(superstructure.intakeAlgaeHigh());
        dpadDown.and(auto).onTrue(superstructure.intakeAlgaeLow());
        operator_controller.povLeft().and(auto).onTrue(superstructure.holdAlgae());
        operator_controller.rightBumper().and(auto).onTrue(superstructure.goToBarge()); // RT then fires it

        // D-pad right: swing the arm to the safe travel angle, in place. It is
        // deliberately not on Back or Start: those are the disabled-only
        // encoder zeros, and a button still held from zeroing when the robot
        // enables would fire an enabled binding without a fresh press.
        operator_controller.povRight().and(auto).onTrue(superstructure.raiseArm());

        // ---- Encoder zeroing: DISABLED only, and only after a 1 s hold ----
        operator_controller.back().and(disabled).debounce(1.0)
            .onTrue(Commands.runOnce(elevator::resetEncoders, elevator).ignoringDisable(true));
        operator_controller.start().and(disabled).debounce(1.0)
            .onTrue(Commands.runOnce(coral::resetPivotEncoder, coral).ignoringDisable(true));

        // ---- Manual take-over (the mechanisms have no default commands, so
        // the sticks are live only while LB is held) ----
        // Pressing LB cancels the running sequence and freezes both
        // mechanisms; sticks jog while it is held; releasing holds position.
        // (and enabled, so LB already held when the robot enables still starts
        // it - whileTrue alone waits for an edge that was dropped while
        // disabled, and with LB held every other operator input is locked out.)
        manual.and(new Trigger(DriverStation::isEnabled)).whileTrue(superstructure.manualOverride(
            () -> -operator_controller.getLeftY(),      // up = carriage up
            () -> -operator_controller.getRightY()));   // forward = claw forward (+ angle)
        // Raw rollers inside manual: no pose check, no subsystem requirement.
        manual.and(operator_controller.rightTrigger())
            .whileTrue(superstructure.rollersRaw(Constants.CorAlConstants.CORAL_SCORE_SPEED));
        manual.and(operator_controller.leftTrigger())
            .whileTrue(superstructure.rollersRaw(Constants.CorAlConstants.ALGAE_INTAKE_SPEED));

        // ---- Rumble cues ----
        // Coral acquired: both drivers, one long buzz -> leave the station.
        new Trigger(coral::isGamePieceDetected).and(teleop)
            .onTrue(driverRumble.pulse(1.0, 0.4).alongWith(operatorRumble.pulse(1.0, 0.4)));
        // Pose reached: operator, two short buzzes -> the score trigger is live.
        ready.and(teleop).onTrue(operatorRumble.pulses(2, 0.8, 0.12, 0.10));
        // Aligned and pose reached: driver, steady light buzz -> fire.
        ready.and(() -> swerve.isVisionTrackingEnabled() && swerve.isAligned())
            .whileTrue(driverRumble.whileActive(0.5));
        // Scored at L3/L4, exit is waiting for room: driver, slow pulse -> back away.
        new Trigger(() -> superstructure.isWaitingForBackOff() || superstructure.isWaitingForReefClearance())
            .whileTrue(driverRumble.pulses(2, 0.6, 0.15, 0.35).repeatedly());
        // Score pulled with nothing to score from: operator, one tick.
        operator_controller.rightTrigger().and(auto).and(() -> !superstructure.isScoringGoal())
            .onTrue(operatorRumble.pulse(0.4, 0.08));
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
     * The trajectory is followed by the same PathPlanner AutoBuilder holonomic
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
                swerve.resetPoseForAuto(AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(start) : start);
            }),
            AutoBuilder.followPath(path)
        );
    }

    /**
     * Called from Robot.disabledExit(): hold both mechanisms where they are.
     * Disabling cuts their outputs, and nothing else commands them again
     * until the operator presses something. Without this hold, a carriage
     * that is up when the robot enables (after an auto that ended at a
     * scoring pose, say) would sink under gravity with the arm un-held,
     * through heights its angle is not clear at, with no interlock running.
     */
    public void enabledInit() {
        elevator.holdCurrentPosition();
        coral.setPivotAngle(coral.getPivotAngle());
    }

    /** Called from Robot.disabledInit(): stop every mechanism output. */
    public void disabledInit() {
        elevator.stopElevator();
        coral.stopPivot();
        coral.stopIntake();
    }
}
