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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Swerve drivetrain subsystem, extending the Phoenix 6 SwerveDrivetrain so the
 * high-frequency odometry loop, module control, and pose estimation all run
 * inside CTRE's optimized code while this class layers on team behavior:
 *
 *  - PathPlanner AutoBuilder configuration for autonomous path following
 *  - Vision pose fusion (measurements arrive via addVisionMeasurement, with
 *    the FPGA-to-Phoenix timestamp conversion handled here)
 *  - A simple AprilTag tracking command that drives toward the best tag
 *  - SysId characterization routines (translation, steer, rotation)
 *  - Operator-perspective handling so field-centric driving matches the
 *    driver's point of view on both alliances
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
	// Keep track if we've ever applied the operator perspective before or not
	private boolean m_hasAppliedOperatorPerspective = false;

	// Swerve request to apply during robot-centric path following.
	// Closed-loop velocity is required for accurate path tracking: each
	// module drives to true ground speed regardless of battery voltage.
	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
		.withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity);

	// Swerve request reused by the AprilTag tracking command (avoids allocating a new request every loop)
	private final SwerveRequest.RobotCentric m_visionTrackRequest = new SwerveRequest.RobotCentric();

	// Swerve requests to apply during SysId characterization
	private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

	// Vision subsystem for AprilTag tracking
	private Vision vision;
	
	// Track vision tracking state internally
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
			// This is in radians per secondÂ², but SysId only supports "volts per second"
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
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
				() -> getState().Pose,      // Supplier of current robot pose
				this::resetPose,            // Consumer for seeding pose against auto
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
	 * @param request Function returning the request to apply
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
	 * test-session utility: the back/start + X/Y bindings run whichever
	 * routine is selected here (translation by default), so call this from
	 * test code or a temporary binding to characterize steer or rotation.
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
	 * Enable or disable vision tracking
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
	 * enabled, drives the robot toward the CLOSEST visible trackable tag
	 * until it sits at the resolved alignment goal (which depends on the
	 * tag's class, the superstructure's goal, and the selected reef branch
	 * side - see Vision.getTrackingGoal), rotating to face the tag.
	 *
	 * Coordinate frames:
	 *   Camera space: X = right (m), Z = forward (m), tx = degrees (+ right)
	 *   Robot space (WPILib): +X = forward, +Y = LEFT, +omega = CCW
	 * Rear-mounted cameras (the coral station camera) see the world rotated
	 * 180 degrees, which exactly negates all three drive terms - handled by
	 * the target's facingSign.
	 *
	 * If no trackable tag is visible (or the target is lost mid-approach)
	 * the command actively commands zero velocity - swerve requests latch,
	 * so without this the robot would keep driving at its last commanded
	 * speed.
	 */
	public Command createAprilTagTrackingCommand() {
		return run(() -> {
			Optional<Vision.AprilTagTarget> target =
				isVisionTrackingEnabled ? vision.getBestTarget() : Optional.empty();
			Optional<Vision.TrackingGoal> goal =
				target.flatMap(tag -> vision.getTrackingGoal(tag.id));

			if (target.isEmpty() || goal.isEmpty()) {
				// No trackable target (or tracking off while still scheduled): stop.
				setControl(m_visionTrackRequest
					.withVelocityX(0)
					.withVelocityY(0)
					.withRotationalRate(0));
				return;
			}

			Vision.AprilTagTarget tag = target.get();

			// Errors between where the tag is and where we want it to be.
			// Positive distanceError = too far away -> close the distance.
			// Positive lateralError = tag is right of the goal in the image.
			double distanceError = tag.poseZ - goal.get().distance;
			double lateralError = tag.poseX - goal.get().lateral;
			double angleError = tag.tx; // Degrees; positive = tag to the right

			// Deadbands prevent hunting around the goal position
			if (Math.abs(distanceError) < VisionConstants.TrackingGains.POSITION_ERROR_DEADBAND) distanceError = 0;
			if (Math.abs(lateralError) < VisionConstants.TrackingGains.POSITION_ERROR_DEADBAND) lateralError = 0;
			if (Math.abs(angleError) < VisionConstants.TrackingGains.ROTATION_ERROR_DEADBAND) angleError = 0;

			// Proportional control mapped into robot-relative velocities.
			// For a front camera: vy is negated because camera X is
			// right-positive but robot Y is left-positive, and omega is
			// negated because a tag to the right needs a clockwise (negative)
			// rotation. A rear camera mirrors all three, so every term is
			// scaled by the camera's facing sign.
			double vx = tag.facingSign * distanceError * VisionConstants.TrackingGains.DISTANCE_kP;
			double vy = tag.facingSign * -lateralError * VisionConstants.TrackingGains.DISTANCE_kP;
			double omega = tag.facingSign * -angleError * VisionConstants.TrackingGains.ROTATION_kP;

			// Clamp velocities to safe tracking limits
			vx = Math.min(Math.max(vx, -VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY), VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY);
			vy = Math.min(Math.max(vy, -VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY), VisionConstants.TrackingGains.MAX_LINEAR_VELOCITY);
			omega = Math.min(Math.max(omega, -VisionConstants.TrackingGains.MAX_ANGULAR_VELOCITY), VisionConstants.TrackingGains.MAX_ANGULAR_VELOCITY);

			setControl(m_visionTrackRequest
				.withVelocityX(vx)
				.withVelocityY(vy)
				.withRotationalRate(omega));
		});
	}

	/**
	 * Adds a vision pose measurement to the drivetrain's pose estimator.
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
		// Skip measurements taken while spinning fast; vision poses degrade badly
		boolean rotatingTooFast = Math.abs(getState().Speeds.omegaRadiansPerSecond) > 2.0;

		if (!rotatingTooFast) {
			super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestampSeconds), stdDevs);
		}
	}

	/** The Vision subsystem owned by this drivetrain (used by the Dashboard). */
	public Vision getVision() {
		return vision;
	}

	@Override
	public void periodic() {
		// Apply operator perspective if not already applied
		if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(
					allianceColor == Alliance.Red
						? kRedAlliancePerspectiveRotation
						: kBlueAlliancePerspectiveRotation
				);
				m_hasAppliedOperatorPerspective = true;
			});
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