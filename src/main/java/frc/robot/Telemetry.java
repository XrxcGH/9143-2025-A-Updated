package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Publishes the swerve drivetrain's state for live dashboards and post-match
 * analysis. Registered with the drivetrain in RobotContainer via
 * {@code swerve.registerTelemetry(logger::telemeterize)}, so it runs at the
 * odometry update rate (not the 20 ms robot loop).
 *
 * Data goes to three places:
 *  - NetworkTables structs ("DriveState" table) for AdvantageScope/Elastic
 *  - CTRE SignalLogger (.hoot files on the roboRIO) for offline analysis
 *  - Mechanism2d widgets visualizing each module's angle and speed
 */
public class Telemetry {
	private final double MaxSpeed;

	/**
	 * Construct a telemetry object, with the specified max speed of the robot.
	 * Also starts the CTRE SignalLogger so every status signal on the CAN bus
	 * is captured to the roboRIO for post-match analysis.
	 *
	 * @param maxSpeed Maximum speed in meters per second
	 */
	public Telemetry(double maxSpeed) {
		MaxSpeed = maxSpeed;
		SignalLogger.start();

		// Register the module Mechanism2d widgets ONCE. telemeterize() runs
		// at the odometry rate (100-250 Hz) on CTRE's odometry thread;
		// re-registering there means hundreds of synchronized dashboard
		// lookups and string allocations per second for no benefit.
		for (int i = 0; i < 4; ++i) {
			SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
		}
	}

	// What to publish over networktables for telemetry
	private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

	// Robot swerve drive state
	private final NetworkTable driveStateTable = inst.getTable("DriveState");
	private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
	private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
	private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
	private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
	private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

	// NOTE: the robot pose for dashboards is published as a proper Field2d by
	// the Dashboard class; this class only handles logging-oriented output.

	// Mechanisms to represent the swerve module states
	private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
		new Mechanism2d(1, 1),
		new Mechanism2d(1, 1),
		new Mechanism2d(1, 1),
		new Mechanism2d(1, 1),
	};
	// A direction and length changing ligament for speed representation
	private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
		m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
		m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
		m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
		m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
	};
	// A direction changing and length constant ligament for module direction
	private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
		m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
		m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
		m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
		m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
	};

	private final double[] m_poseArray = new double[3];
	private final double[] m_moduleStatesArray = new double[8];
	private final double[] m_moduleTargetsArray = new double[8];

	/**
	 * Publishes one drivetrain state sample to NetworkTables, the SignalLogger
	 * and the module Mechanism2d widgets. Called on CTRE's odometry thread.
	 */
	public void telemeterize(SwerveDriveState state) {
		// Telemeterize the swerve drive state
		drivePose.set(state.Pose);
		driveSpeeds.set(state.Speeds);
		driveModuleStates.set(state.ModuleStates);
		driveModuleTargets.set(state.ModuleTargets);
		driveModulePositions.set(state.ModulePositions);
		driveTimestamp.set(state.Timestamp);
		driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

		// Also write to log file
		m_poseArray[0] = state.Pose.getX();
		m_poseArray[1] = state.Pose.getY();
		m_poseArray[2] = state.Pose.getRotation().getDegrees();
		for (int i = 0; i < 4; ++i) {
			m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
			m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
			m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
			m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
		}

		SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
		SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
		SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
		SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

		// Telemeterize the module states to the (constructor-registered)
		// Mechanism2d widgets
		for (int i = 0; i < 4; ++i) {
			m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
			m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
			m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
		}
	}
}