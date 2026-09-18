package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Tunables;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Swerve drivetrain subsystem, extending the Phoenix 6 SwerveDrivetrain so the
 * high-frequency odometry loop, module control, and pose estimation all run
 * inside CTRE's optimized code while this class layers on team behavior:
 *
 *  - PathPlanner AutoBuilder configuration for autonomous path following
 *  - Vision pose fusion (measurements arrive via addVisionMeasurement, with
 *    the FPGA-to-Phoenix timestamp conversion handled here)
 *  - The AprilTag alignment command: a robot-frame servo that drives the
 *    tracked tag to the goal position Vision resolves (reef branch, coral
 *    station, barge, processor) and squares the robot to the tag's face
 *  - SysId characterization routines (translation, steer, rotation)
 *  - Operator-perspective handling so field-centric driving matches the
 *    driver's point of view on both alliances, with the driver's forward
 *    direction held in the raw gyro frame so vision heading corrections
 *    never move it (see periodic())
 *
 * The internal Vision subsystem is constructed here; do not create another
 * Vision instance elsewhere.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	// Blue alliance sees forward as 0 degrees (toward red alliance wall)
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	// Red alliance sees forward as 180 degrees (toward blue alliance wall)
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
	// The DRIVER's forward direction, held in the RAW gyro frame (the one
	// thing vision corrections and pose resets never touch). See periodic().
	private Rotation2d m_driverForwardRaw = null;
	private Alliance m_driverForwardAlliance = null;
	/** True once the DRIVER has zeroed their heading: from then on vision never moves it. */
	private boolean m_driverZeroed = false;

	// Swerve request to apply during robot-centric path following.
	// Closed-loop velocity is required for accurate path tracking: each
	// module drives to true ground speed regardless of battery voltage.
	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
		.withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

	// Swerve request reused by the AprilTag tracking command (avoids allocating
	// a new request every loop). Closed-loop velocity like every other drive
	// request: the small commands the alignment servo produces near its
	// deadbands would never overcome static friction open-loop.
	private final SwerveRequest.RobotCentric m_visionTrackRequest = new SwerveRequest.RobotCentric()
		.withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

	// Alignment telemetry (robot frame: meters / degrees), refreshed by the tracking command
	private boolean m_alignHasTarget = false;
	private boolean m_aligned = false;
	private double m_alignForwardError = 0.0;
	private double m_alignLateralError = 0.0;
	private double m_alignHeadingErrorDeg = 0.0;
	// Tracking command state: the last velocities SENT (the slew limiter works
	// from these), the stay-stopped latches, and the contact-stall timer
	private double m_trackVx = 0.0;
	private double m_trackVy = 0.0;
	private double m_trackOmega = 0.0;
	private double m_trackLastTime = 0.0;
	private boolean m_trackTranslationHeld = false;
	private boolean m_trackContactHeld = false; // held because the bumper is ON the reef, not because the error is small
	private boolean m_trackHeadingHeld = false;
	private double m_trackStalledSince = -1.0;
	// Whether the last autonomous pose reset kept the vision-seeded heading
	private boolean m_lastAutoResetKeptHeading = false;

	// Vision-measurement spin rejection: angular rate above which vision
	// poses are untrustworthy, and how long after the spin ends they stay
	// rejected (covers image capture latency of the last smeared frames)
	private static final double kVisionMaxOmegaRadPerSec = 2.0;
	private static final double kVisionRejectAfterSpinSeconds = 0.2;
	private double m_lastFastRotationTime = -kVisionRejectAfterSpinSeconds;

	// Swerve requests to apply during SysId characterization
	private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

	// Vision subsystem (pose fusion and AprilTag alignment targets)
	private Vision vision;

	// True while an alignment is in progress: set by the hold-to-align
	// bindings, cleared whenever the tracking command ends
	private boolean isVisionTrackingEnabled = false;

	public Command aprilTagTrackingCommand;

	// SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
	private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,   		// Use default ramp rate (1 V/s)
			Volts.of(4),	// Reduce dynamic step voltage to 4 V to prevent brownout
			null,   		// Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> setControl(m_translationCharacterization.withVolts(output)),
			null,
			this
		)
	);

	// SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
	private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,			// Use default ramp rate (1 V/s)
			Volts.of(7),	// Use dynamic voltage of 7 V
			null,			// Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			volts -> setControl(m_steerCharacterization.withVolts(volts)),
			null,
			this
		)
	);

	/*
	 * SysId routine for characterizing rotation.
	 * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
	 * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
	 */
	private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
		new SysIdRoutine.Config(
			// This is in radians per second^2, but SysId only supports "volts per second"
			Volts.of(Math.PI / 6).per(Second),
			// This is in radians per second, but SysId only supports "volts"
			Volts.of(Math.PI),
			null,	// Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
		),
		new SysIdRoutine.Mechanism(
			output -> {
				/* output is actually radians per second, but SysId only supports "volts" */
				setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
				/* also log the requested output for SysId */
				SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
			},
			null,
			this
		)
	);

	// The SysId routine to test
	private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
	 * @param modules               Constants for each specific module
	 */
	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, modules);

		this.vision = new Vision(this);

		if (Utils.isSimulation()) {
			startSimThread();
		}

		aprilTagTrackingCommand = createAprilTagTrackingCommand();

		configureAutoBuilder();
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency The frequency to run the odometry loop
	 * @param modules                 Constants for each specific module
	 */
	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants,
		double odometryUpdateFrequency,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, odometryUpdateFrequency, modules);

		this.vision = new Vision(this);

		if (Utils.isSimulation()) {
			startSimThread();
		}

		aprilTagTrackingCommand = createAprilTagTrackingCommand();

		configureAutoBuilder();
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency   The frequency to run the odometry loop
	 * @param odometryStandardDeviation The standard deviation for odometry calculation
	 * @param visionStandardDeviation   The standard deviation for vision calculation
	 * @param modules                   Constants for each specific module
	 */
	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants,
		double odometryUpdateFrequency,
		Matrix<N3, N1> odometryStandardDeviation,
		Matrix<N3, N1> visionStandardDeviation,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

		this.vision = new Vision(this);

		if (Utils.isSimulation()) {
			startSimThread();
		}

		aprilTagTrackingCommand = createAprilTagTrackingCommand();

		configureAutoBuilder();
	}

	private void configureAutoBuilder() {
		// PathPlannerLib 2026 defaults its alliance flip to the 2026 field size
		// (16.54 x 8.07 m); this robot plays on the 2025 Reefscape field
		// (17.548 x 8.052 m, matching navgrid.json and the tag layout). Without
		// this every red-alliance path, odometry reset and Choreo start pose
		// would be mirrored about the wrong centerline, about 1 m off in X.
		FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
		FlippingUtil.fieldSizeX = VisionConstants.FIELD_LENGTH_METERS;
		FlippingUtil.fieldSizeY = VisionConstants.FIELD_WIDTH_METERS;
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
				() -> getState().Pose,      // Supplier of current robot pose
				this::resetPoseForAuto,     // Consumer for seeding pose against auto (keeps a fresh vision heading)
				() -> getState().Speeds,    // Supplier of current robot speeds
				// Consumer of ChassisSpeeds and feedforwards to drive the robot
				(speeds, feedforwards) -> setControl(
					m_pathApplyRobotSpeeds.withSpeeds(speeds)
						.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
						.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
				),
				new PPHolonomicDriveController(
					// Feedback gains that pull the robot back onto the path;
					// PathPlanner supplies the feedforward from the path itself
					new PIDConstants(AutoConstants.TRANSLATION_kP, AutoConstants.TRANSLATION_kI, AutoConstants.TRANSLATION_kD),
					new PIDConstants(AutoConstants.ROTATION_kP, AutoConstants.ROTATION_kI, AutoConstants.ROTATION_kD)
				),
				config,
				// Assume the path needs to be flipped for Red vs Blue, this is normally the case
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this
			);
		} catch (Exception ex) {
			DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
		}
	}

	/**
	 * Returns a command that applies the specified control request to this swerve drivetrain.
	 *
	 * @param requestSupplier Function returning the request to apply
	 * @return Command to run
	 */
	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.dynamic(direction);
	}

	/**
	 * Set the currently active SysId routine to use.
	 *
	 * NOTE: intentionally not called anywhere by default. This is a
	 * test-session utility: the driver's Test-mode Back / Start + X / Y
	 * bindings run whichever routine is selected here (translation by
	 * default), so call this from test code or a temporary binding to
	 * characterize steer or rotation.
	 *
	 * @param routineType The type of SysId routine to use
	 */
	public void setSysIdRoutine(SysIdRoutineType routineType) {
		switch (routineType) {
			case TRANSLATION:
				m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
				break;
			case STEER:
				m_sysIdRoutineToApply = m_sysIdRoutineSteer;
				break;
			case ROTATION:
				m_sysIdRoutineToApply = m_sysIdRoutineRotation;
				break;
		}
	}
	
	/**
	 * Enum for selecting which SysId routine to use
	 */
	public enum SysIdRoutineType {
		TRANSLATION,
		STEER,
		ROTATION
	}

	/**
	 * Enables or disables vision tracking. Enabling only arms the tracking
	 * command (see createAprilTagTrackingCommand); disabling also releases
	 * Vision's tag latch.
	 *
	 * @param enabled Whether vision tracking should be enabled
	 */
	public void setVisionTrackingEnabled(boolean enabled) {
		isVisionTrackingEnabled = enabled;
		vision.toggleTracking(enabled);
	}
	
	/**
	 * Get whether vision tracking is currently enabled
	 * 
	 * @return Whether vision tracking is enabled
	 */
	public boolean isVisionTrackingEnabled() {
		return isVisionTrackingEnabled;
	}

	/**
	 * Creates the AprilTag tracking command: while vision tracking is
	 * enabled, drives the robot toward the latched/closest trackable tag
	 * until the tag sits at the resolved alignment goal in the ROBOT frame
	 * (which depends on the tag's class, the superstructure's goal, and the
	 * selected reef branch side - see Vision.getTrackingGoal) and the robot
	 * is square to the tag's face.
	 *
	 * Frames: Vision converts each camera's tag position into the robot
	 * frame (+X forward, +Y left) using that camera's mounting pose, so a
	 * rear camera, a yawed camera or an offset lens all drive the same loop:
	 * vx closes the forward error, vy the lateral error, and omega turns the
	 * estimated field heading to the heading that is square to the tag's
	 * face (from the field layout; if the tag is not in the layout, the
	 * bearing to the goal point is used instead). Nothing is mirrored per
	 * camera.
	 *
	 * Translation is ONE proportional controller on the error vector (speed
	 * from its length, with a minimum so the last centimeters are driven
	 * rather than lost to static friction; direction along it), heading is a
	 * second; both stop inside their deadbands and stay stopped until the
	 * error grows past the exit ratio, and everything goes through a slew
	 * limiter. The lateral deadband is tighter than the forward one because
	 * a coral has only ~3 cm of lateral clearance on a branch. The tag
	 * position it servos on is Vision's filtered, latency-free estimate, not
	 * a raw camera solve - see the Vision class note.
	 *
	 * If no trackable tag is visible (or the target is lost mid-approach)
	 * the command ramps to zero velocity - swerve requests latch, so without
	 * this the robot would keep driving at its last commanded speed. When the
	 * command ends for any reason (the driver releases the align trigger, or
	 * another swerve command interrupts it), it stops the robot and clears
	 * the tracking flag, so isVisionTrackingEnabled() always reflects whether
	 * an alignment is actually running.
	 */
	public Command createAprilTagTrackingCommand() {
		return startRun(() -> {
			// Start the slew limiter from what the robot is actually doing, so
			// taking over from the driver mid-motion is continuous too.
			var speeds = getStateCopy().Speeds;
			m_trackVx = speeds.vxMetersPerSecond;
			m_trackVy = speeds.vyMetersPerSecond;
			m_trackOmega = speeds.omegaRadiansPerSecond;
			m_trackTranslationHeld = false;
			m_trackContactHeld = false;
			m_trackHeadingHeld = false;
			m_trackStalledSince = -1.0;
		}, () -> {
			Optional<Vision.AprilTagTarget> target =
				isVisionTrackingEnabled ? vision.getBestTarget() : Optional.empty();
			Optional<Vision.TrackingGoal> goal = target.flatMap(vision::getTrackingGoal);

			if (target.isEmpty() || goal.isEmpty()) {
				// No trackable target (or tracking off while still scheduled):
				// come to a stop - ramped, not stepped.
				clearAlignmentTelemetry();
				m_trackTranslationHeld = false;
				m_trackContactHeld = false;
				m_trackHeadingHeld = false;
				m_trackStalledSince = -1.0;
				applyTrackingCommand(0.0, 0.0, 0.0);
				return;
			}

			Vision.AprilTagTarget tag = target.get();
			Vision.TrackingGoal g = goal.get();

			// Errors between where the tag is and where we want it (robot frame).
			// Positive forward error = tag further ahead than wanted -> drive forward.
			// Positive lateral error = tag further left than wanted -> drive left.
			double forwardError = tag.robotFrame.getX() - g.forward;
			double lateralError = tag.robotFrame.getY() - g.left;
			// Heading: square to the tag's face when its field pose is known,
			// otherwise point the robot at the goal point (bearing servo).
			Rotation2d heading = getStateCopy().Pose.getRotation();
			double headingErrorDeg = tag.squareHeading
				.map(square -> square.minus(heading).getDegrees())
				.orElseGet(() -> new Rotation2d(tag.robotFrame.getX(), tag.robotFrame.getY())
					.minus(new Rotation2d(g.forward, g.left)).getDegrees());

			m_alignHasTarget = true;
			m_alignForwardError = forwardError;
			m_alignLateralError = lateralError;
			m_alignHeadingErrorDeg = headingErrorDeg;

			// ---- Translation: ONE controller on the error VECTOR ----
			// The speed comes from the length of the error vector and the
			// direction is simply along it, so the direction of travel turns
			// smoothly all the way in; the robot stops when BOTH axes are
			// inside their deadbands and stays stopped until one grows past
			// the exit ratio. Per-axis controllers, each with its own deadband
			// and minimum speed, must not be used here: as the axes cross
			// their deadbands at different moments the commanded velocity
			// snaps between (min, 0), (min, min) and (0, min) - the DIRECTION
			// of travel jumping 45-90 deg with the robot nearly stationary,
			// which makes every swerve module whip round to a new steering
			// angle and jerks the whole robot.
			double forwardDeadband = VisionConstants.TrackingGains.FORWARD_ERROR_DEADBAND;
			double lateralDeadband = VisionConstants.TrackingGains.LATERAL_ERROR_DEADBAND;
			double exitRatio = VisionConstants.TrackingGains.DEADBAND_EXIT_RATIO;
			boolean inside = Math.abs(forwardError) < forwardDeadband
				&& Math.abs(lateralError) < lateralDeadband;
			// Held by CONTACT the forward error is, by definition, outside the
			// ordinary exit band (that is why contact was needed), so it gets
			// its own: without it the latch set below would be undone on the
			// very next loop, and the robot would shove the reef again every
			// CONTACT_SECONDS.
			double forwardExit = m_trackContactHeld
				? VisionConstants.TrackingGains.CONTACT_FORWARD_ERROR * exitRatio : forwardDeadband * exitRatio;
			boolean outside = Math.abs(forwardError) > forwardExit
				|| Math.abs(lateralError) > lateralDeadband * exitRatio;
			if (m_trackTranslationHeld ? outside : inside) {
				m_trackTranslationHeld = !m_trackTranslationHeld;
				m_trackContactHeld = false;
			}

			// Arrived by CONTACT: told to move, not moving, laterally in
			// position and nearly there forward - the bumper is on the reef
			// (or the wall). Pushing on would only stall the drivetrain.
			var measured = getStateCopy().Speeds;
			double measuredSpeed = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
			double now = Timer.getFPGATimestamp();
			boolean pushing = !m_trackTranslationHeld
				&& Math.hypot(m_trackVx, m_trackVy) >= VisionConstants.TrackingGains.MIN_LINEAR_VELOCITY * 0.9
				&& measuredSpeed < VisionConstants.TrackingGains.CONTACT_MAX_SPEED
				&& Math.abs(lateralError) < lateralDeadband * exitRatio
				&& Math.abs(forwardError) < VisionConstants.TrackingGains.CONTACT_FORWARD_ERROR;
			if (!pushing) {
				m_trackStalledSince = -1.0;
			} else if (m_trackStalledSince < 0) {
				m_trackStalledSince = now;
			} else if (now - m_trackStalledSince > VisionConstants.TrackingGains.CONTACT_SECONDS) {
				m_trackTranslationHeld = true;
				m_trackContactHeld = true;
			}

			double vx = 0.0;
			double vy = 0.0;
			if (!m_trackTranslationHeld) {
				double distance = Math.hypot(forwardError, lateralError);
				// Gains are live-tunable from the dashboard (Tunables -> Preferences)
				double speed = Math.min(Math.max(distance * Tunables.trackingDistanceKp(),
					VisionConstants.TrackingGains.MIN_LINEAR_VELOCITY), VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY);
				if (distance > 1e-6) {
					vx = speed * forwardError / distance;
					vy = speed * lateralError / distance;
				}
			}

			// ---- Heading: P with the same stay-stopped hysteresis ----
			double headingDeadband = VisionConstants.TrackingGains.ROTATION_ERROR_DEADBAND;
			if (m_trackHeadingHeld
					? Math.abs(headingErrorDeg) > headingDeadband * exitRatio
					: Math.abs(headingErrorDeg) < headingDeadband) {
				m_trackHeadingHeld = !m_trackHeadingHeld;
			}
			double omega = m_trackHeadingHeld ? 0.0
				: Math.copySign(Math.min(Math.abs(headingErrorDeg) * Tunables.trackingRotationKp(),
					VisionConstants.TrackingGains.MAX_ANGULAR_VELOCITY), headingErrorDeg);

			m_aligned = m_trackTranslationHeld && m_trackHeadingHeld;
			applyTrackingCommand(vx, vy, omega);
		}).finallyDo(() -> {
			// Runs on cancel (align trigger released) AND on interruption by
			// any other swerve command (brake, point, D-pad nudges, SysId):
			// stop the robot and drop the tracking state so the flag always
			// reflects reality.
			isVisionTrackingEnabled = false;
			vision.toggleTracking(false);
			clearAlignmentTelemetry();
			m_trackVx = 0.0;
			m_trackVy = 0.0;
			m_trackOmega = 0.0;
			setControl(m_visionTrackRequest
				.withVelocityX(0)
				.withVelocityY(0)
				.withRotationalRate(0));
		});
	}

	/** No target: the error readouts must not freeze at their last values. */
	private void clearAlignmentTelemetry() {
		m_alignHasTarget = false;
		m_aligned = false;
		m_alignForwardError = 0.0;
		m_alignLateralError = 0.0;
		m_alignHeadingErrorDeg = 0.0;
	}

	/**
	 * Output stage of the tracking command: slew-limits the commanded
	 * velocity VECTOR (so its direction turns as well as its length ramps)
	 * and the rotation rate, then sends them. Nothing the servo decides can
	 * reach the modules as a step.
	 */
	private void applyTrackingCommand(double vx, double vy, double omega) {
		double now = Timer.getFPGATimestamp();
		double dt = Math.min(Math.max(now - m_trackLastTime, 0.005), 0.05);
		m_trackLastTime = now;

		double dvx = vx - m_trackVx;
		double dvy = vy - m_trackVy;
		double change = Math.hypot(dvx, dvy);
		double maxChange = VisionConstants.TrackingGains.MAX_LINEAR_ACCELERATION * dt;
		double scale = change > maxChange ? maxChange / change : 1.0;
		m_trackVx += dvx * scale;
		m_trackVy += dvy * scale;

		double maxOmegaChange = VisionConstants.TrackingGains.MAX_ANGULAR_ACCELERATION * dt;
		m_trackOmega += Math.max(-maxOmegaChange, Math.min(maxOmegaChange, omega - m_trackOmega));

		setControl(m_visionTrackRequest
			.withVelocityX(m_trackVx)
			.withVelocityY(m_trackVy)
			.withRotationalRate(m_trackOmega));
	}

	/**
	 * True while vision measurements are being rejected: the robot has spun
	 * faster than the limit within the last kVisionRejectAfterSpinSeconds
	 * (motion blur and rolling shutter corrupt the solve; the image was
	 * captured 25-100 ms ago, so the gate covers the last smeared frames
	 * after a spin ends, not just this instant). Vision checks this so it
	 * never books a fusion that did not happen.
	 */
	public boolean isRejectingVision() {
		return Timer.getFPGATimestamp() - m_lastFastRotationTime < kVisionRejectAfterSpinSeconds;
	}

	/**
	 * Adds a vision pose measurement to the drivetrain's pose estimator.
	 * Measurements are dropped while {@link #isRejectingVision()} is true.
	 *
	 * @param visionPose       robot pose on the field as seen by vision
	 * @param timestampSeconds capture timestamp in the FPGA timebase
	 *                         (e.g. from LimelightHelpers); it is converted to
	 *                         the Phoenix 6 timebase here, which the CTRE
	 *                         swerve pose estimator requires. Passing raw FPGA
	 *                         time makes every measurement appear to be from
	 *                         the wrong moment and corrupts the pose estimate.
	 * @param stdDevs          measurement standard deviations [x, y, theta]
	 */
	@Override
	public void addVisionMeasurement(
		Pose2d visionPose,
		double timestampSeconds,
		Matrix<N3, N1> stdDevs) {
		if (isRejectingVision()) {
			return;
		}
		super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestampSeconds), stdDevs);
	}

	/** The Vision subsystem owned by this drivetrain (used by the Dashboard). */
	public Vision getVision() {
		return vision;
	}

	/**
	 * Pose reset used at the start of every auto (PathPlanner's resetOdom and
	 * the Choreo autos). The path's ideal starting pose is a nominal
	 * placement; the heading the pose estimator holds when a strong MegaTag1
	 * seed is fresh (two or more tags while sitting on the line) is the real
	 * one, and it is the heading MegaTag2 trusts absolutely for the whole
	 * period - overwriting it with the nominal value would bias every vision
	 * pose by the placement error. So: keep the vision heading and reset only
	 * the translation when the seed is strong and agrees with the nominal
	 * heading within HEADING_SEED_MAX_DISAGREEMENT_DEGREES; otherwise reset
	 * the full pose (translation and heading) to the nominal start.
	 */
	public void resetPoseForAuto(Pose2d nominalStart) {
		Rotation2d current = getStateCopy().Pose.getRotation();
		double disagreementDeg = Math.abs(current.minus(nominalStart.getRotation()).getDegrees());
		// Either way the pose heading is now the field's, so the driver's
		// forward becomes the alliance's forward in it - computed HERE from
		// the rotation the pose is being given, not next loop from the cached
		// state (which may not show the reset yet).
		Alliance alliance = DriverStation.getAlliance().orElse(
			m_driverForwardAlliance != null ? m_driverForwardAlliance : Alliance.Blue);
		Rotation2d rawHeading = getStateCopy().RawHeading;
		if (vision.hasStrongHeadingSeed()
				&& disagreementDeg <= VisionConstants.HEADING_SEED_MAX_DISAGREEMENT_DEGREES) {
			resetTranslation(nominalStart.getTranslation());
			m_lastAutoResetKeptHeading = true;
			m_driverForwardRaw = allianceForward(alliance).minus(current.minus(rawHeading));
		} else {
			resetPose(nominalStart);
			m_lastAutoResetKeptHeading = false;
			m_driverForwardRaw = allianceForward(alliance).minus(nominalStart.getRotation().minus(rawHeading));
		}
		m_driverForwardAlliance = alliance;
		m_driverZeroed = false;
		setOperatorPerspectiveForward(allianceForward(alliance));
	}

	/** Whether the last autonomous pose reset kept the vision-seeded heading. */
	public boolean lastAutoResetKeptHeading() {
		return m_lastAutoResetKeptHeading;
	}

	/** Whether the alignment servo currently has a target. */
	public boolean isAlignmentTargetVisible() {
		return m_alignHasTarget;
	}

	/** True while the tracker has a target and every axis is inside its deadband. */
	public boolean isAligned() {
		return m_alignHasTarget && m_aligned;
	}

	public double getAlignmentForwardError() {
		return m_alignForwardError;
	}

	public double getAlignmentLateralError() {
		return m_alignLateralError;
	}

	public double getAlignmentHeadingErrorDegrees() {
		return m_alignHeadingErrorDeg;
	}

	private static Rotation2d allianceForward(Alliance alliance) {
		return alliance == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation;
	}

	/**
	 * Driver heading zero: the way the robot faces NOW is "forward" on the
	 * driver's stick (robot pointing away from the driver - the same on
	 * either alliance). It does not touch the pose estimator, so it is safe
	 * mid-match: MegaTag2 and the autos keep the heading they had.
	 *
	 * @param resetPoseHeading also reset the POSE heading to the alliance's
	 *     forward direction - a field-heading seed for when no tag can supply
	 *     one (robot squared up by hand, no tags in view).
	 */
	public void zeroDriverHeading(boolean resetPoseHeading) {
		m_driverForwardRaw = getStateCopy().RawHeading;
		m_driverZeroed = true;
		if (resetPoseHeading) {
			Rotation2d forward = allianceForward(DriverStation.getAlliance().orElse(Alliance.Blue));
			resetRotation(forward);
			// The pose now faces "forward" where the robot faces, so that IS the
			// perspective - set at once, not a loop late against the new heading.
			setOperatorPerspectiveForward(forward);
		}
	}

	@Override
	public void periodic() {
		var state = getStateCopy();

		// Track when the robot last spun fast, for vision-measurement rejection
		if (Math.abs(state.Speeds.omegaRadiansPerSecond) > kVisionMaxOmegaRadPerSec) {
			m_lastFastRotationTime = Timer.getFPGATimestamp();
		}

		// ---- The driver's field-centric frame is glued to the GYRO ----
		// Field-centric requests steer relative to the POSE heading plus the
		// "operator perspective". The pose heading is vision's to correct:
		// MegaTag1 re-seeds it whenever the robot sits disabled looking at a
		// tag (and in a re-seed window). If the driver's frame followed the
		// pose heading, every re-seed would move it - align on a tag, disable
		// in front of it, re-enable, and "forward" on the stick has moved:
		// the robot drives the wrong way.
		// So the driver's forward is kept as a direction in the RAW gyro
		// frame, which nothing but the gyro moves, and the operator
		// perspective is recomputed every loop to cancel whatever vision or a
		// pose reset did to the pose heading. It changes only when the driver
		// zeroes it (zeroDriverHeading), when an auto resets the pose to the
		// field's frame, when the alliance changes, or - until the driver has
		// zeroed it - while disabled with a converged multi-tag heading seed.
		Rotation2d poseMinusRaw = state.Pose.getRotation().minus(state.RawHeading);
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			boolean changed = m_driverForwardAlliance != null && alliance.get() != m_driverForwardAlliance;
			// Until the driver has zeroed it themselves, a CONVERGED two-or-more
			// tag heading seed while disabled sets the driver's frame, which is
			// what a real field needs (robot booted facing any which way,
			// placed on the field, never zeroed). One reef tag in front of the
			// bumper - the bench case that must not move the driver's frame -
			// is not a strong seed, and after a driver zero nothing from vision
			// counts.
			boolean seeded = DriverStation.isDisabled() && !m_driverZeroed && vision.hasStrongHeadingSeed();
			if (m_driverForwardRaw == null || changed || seeded) {
				m_driverForwardRaw = allianceForward(alliance.get()).minus(poseMinusRaw);
			}
			m_driverForwardAlliance = alliance.get();
		}
		if (m_driverForwardRaw != null) {
			setOperatorPerspectiveForward(m_driverForwardRaw.plus(poseMinusRaw));
		}
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		// Run simulation at a faster rate so PID gains behave more reasonably
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			// use the measured time delta, get battery voltage from WPILib
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}
}