package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

/**
 * Elevator subsystem driven by two NEO brushless motors on Spark MAX
 * controllers, each through a 45:1 MAXPlanetary reduction (5:1 x 3:1 x 3:1
 * cartridges) to the winch drum.
 *
 * Control architecture:
 *  - The left Spark MAX is the leader; the right is configured as a hardware
 *    follower, so both sides always receive identical output and cannot
 *    fight each other.
 *  - The encoder conversion factors scale the NEO's integrated encoder so
 *    every position is in inches and every velocity in inches per second.
 *  - Height moves use MAXMotion (trapezoidal profiling on the controller)
 *    with a constant gravity feedforward (kG) passed as arbitrary
 *    feedforward voltage, so the carriage tracks smoothly and holds its
 *    height at rest and when the operator releases the stick.
 *  - Soft limits on the controller bound travel in every control mode, and
 *    voltage compensation keeps response consistent as the battery sags.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class Elevator extends SubsystemBase {

    private final SparkMax leftMotor;   // Leader
    private final SparkMax rightMotor;  // Follower

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkClosedLoopController closedLoopController;

    // Track target position internally (inches)
    private double currentTargetPosition = 0.0;
    // Track manual mode state internally
    private boolean manualModeEnabled = false;
    // Track position control mode internally
    private boolean positionControlEnabled = false;

    // ------------------------------------------------------------------
    // Desktop simulation (only constructed when running off-robot). The
    // physics model exists purely so the mechanism moves in the sim GUI /
    // AdvantageScope; the values below affect simulation fidelity only.
    // ------------------------------------------------------------------
    // Carriage travels 11 in per drum rotation (5.5 in of chain x2 cascade),
    // modeled as an effective drum radius of 11 in / (2*pi).
    private static final double SIM_DRUM_RADIUS_METERS = Units.inchesToMeters(11.0) / (2.0 * Math.PI);
    private static final double SIM_CARRIAGE_MASS_KG = 6.0; // Estimate - affects sim only
    private SparkMaxSim leftMotorSim;
    private ElevatorSim elevatorSim;

    public Elevator() {
        leftMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);

        configureMotors();

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        // All closed-loop commands go to the leader; the follower mirrors it
        closedLoopController = leftMotor.getClosedLoopController();

        // Reset encoders on initialization (elevator must start at its base position)
        resetEncoders();

        if (RobotBase.isSimulation()) {
            leftMotorSim = new SparkMaxSim(leftMotor, DCMotor.getNEO(1));
            elevatorSim = new ElevatorSim(
                DCMotor.getNEO(2),
                ElevatorConstants.ELEVATOR_GEAR_RATIO,
                SIM_CARRIAGE_MASS_KG,
                SIM_DRUM_RADIUS_METERS,
                Units.inchesToMeters(ElevatorConstants.ELEVATOR_MIN_POSITION),
                Units.inchesToMeters(ElevatorConstants.ELEVATOR_MAX_POSITION),
                true, // Simulate gravity so kG/holding behavior is visible
                Units.inchesToMeters(ElevatorConstants.ELEVATOR_MIN_POSITION));
        }
    }

    /**
     * Builds and applies the leader and follower configurations. Parameters
     * are persisted to flash so a brownout or power cycle cannot silently
     * revert the controllers to factory defaults mid-match.
     */
    private void configureMotors() {
        // --- Shared base configuration ---
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        leaderConfig
            .inverted(ElevatorConstants.ELEVATOR_LEFT_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT)
            .voltageCompensation(ElevatorConstants.ELEVATOR_NOMINAL_VOLTAGE);

        // Scale the NEO encoder so position is in inches and velocity is in
        // inches per second (native units are motor rotations and RPM).
        leaderConfig.encoder
            .positionConversionFactor(ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION)
            .velocityConversionFactor(ElevatorConstants.ELEVATOR_INCHES_PER_ROTATION / 60.0);

        // Closed-loop PID gains (slot 0). Error units are inches after the
        // conversion factors above.
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevatorConstants.ELEVATOR_kP)
            .i(ElevatorConstants.ELEVATOR_kI)
            .d(ElevatorConstants.ELEVATOR_kD)
            .outputRange(-1, 1);

        // On-controller feedforward (volts): static friction, velocity, and
        // gravity compensation - the same kS/kV/kG model as WPILib's
        // ElevatorFeedforward, evaluated by the Spark MAX every cycle so the
        // carriage tracks the MAXMotion profile and holds height at rest.
        leaderConfig.closedLoop.feedForward
            .kS(ElevatorConstants.ELEVATOR_kS)
            .kV(ElevatorConstants.ELEVATOR_kV)
            .kG(ElevatorConstants.ELEVATOR_kG);

        // MAXMotion profile parameters (inches, inches per second)
        leaderConfig.closedLoop.maxMotion
            .cruiseVelocity(ElevatorConstants.ELEVATOR_MAX_VELOCITY)
            .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ACCELERATION)
            .allowedProfileError(ElevatorConstants.ELEVATOR_ALLOWED_ERROR);

        // Soft limits (inches) bound travel in every control mode
        leaderConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.ELEVATOR_MAX_POSITION)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorConstants.ELEVATOR_MIN_POSITION)
            .reverseSoftLimitEnabled(true);

        // --- Follower configuration ---
        // Identical to the leader, except it follows the leader's output.
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.apply(leaderConfig);
        followerConfig.follow(ElevatorConstants.ELEVATOR_LEFT_ID,
            ElevatorConstants.ELEVATOR_RIGHT_OPPOSES_LEFT);

        leftMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Commands the elevator to a height in inches using MAXMotion.
     * The Spark MAX runs the profile and holds the position afterward;
     * gravity/friction compensation comes from the configured kS/kV/kG.
     */
    public void setPosition(double targetPosition) {
        // Clamp target position within safe limits
        targetPosition = Math.min(Math.max(targetPosition, ElevatorConstants.ELEVATOR_MIN_POSITION),
            ElevatorConstants.ELEVATOR_MAX_POSITION);

        currentTargetPosition = targetPosition;
        positionControlEnabled = true;
        manualModeEnabled = false;

        // The controller latches the setpoint; no need to re-send every loop
        closedLoopController.setSetpoint(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    /**
     * Holds the current height under closed-loop control. Used when the driver
     * releases the manual-control stick so the carriage does not drift down.
     */
    public void holdCurrentPosition() {
        setPosition(getCurrentPosition());
    }

    /** Zeros both encoders; only do this with the carriage at its base. */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Direct duty-cycle control from the operator stick. Soft limits on the
     * controllers stop travel at either end of the elevator's range.
     */
    public void manualControl(double speed) {
        // Manual input overrides position control
        positionControlEnabled = false;
        manualModeEnabled = true;

        // Apply deadband and limits
        if (Math.abs(speed) < ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        speed = Math.min(Math.max(speed * ElevatorConstants.ELEVATOR_MANUAL_SPEED_LIMIT, -1), 1);

        leftMotor.set(speed);
    }

    /** Cuts output and drops any closed-loop target (used when disabling). */
    public void stopElevator() {
        positionControlEnabled = false;
        leftMotor.set(0);
    }

    // ------------------------------------------------------------------
    // State getters (used by commands and the Dashboard)
    // ------------------------------------------------------------------

    /** Current height in inches, read from the leader's encoder. */
    public double getCurrentPosition() {
        return leftEncoder.getPosition();
    }

    /** The height in inches the closed loop is targeting. */
    public double getTargetPosition() {
        return currentTargetPosition;
    }

    public boolean isAtTargetPosition() {
        double currentPosition = getCurrentPosition();
        return Math.abs(currentTargetPosition - currentPosition) <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR;
    }

    /**
     * True when both sides report the same position. A persistent mismatch
     * means slippage or a mechanical problem; the Dashboard raises an alert.
     */
    public boolean sidesInSync() {
        return Math.abs(leftEncoder.getPosition() - rightEncoder.getPosition())
            <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR;
    }

    public boolean isInManualMode() {
        return manualModeEnabled;
    }

    /** Leader (left) motor output current in amps, for diagnostics. */
    public double getLeftCurrent() {
        return leftMotor.getOutputCurrent();
    }

    /** Follower (right) motor output current in amps, for diagnostics. */
    public double getRightCurrent() {
        return rightMotor.getOutputCurrent();
    }

    /** Leader (left) motor applied duty cycle, -1 to 1. */
    public double getLeftOutput() {
        return leftMotor.getAppliedOutput();
    }

    /** Follower (right) motor applied duty cycle, -1 to 1. */
    public double getRightOutput() {
        return rightMotor.getAppliedOutput();
    }

    /** Carriage velocity in inches per second. */
    public double getVelocity() {
        return leftEncoder.getVelocity();
    }

    // No periodic() override: closed-loop control runs on the Spark MAX, and
    // all monitoring/alerting is centralized in Dashboard.

    /**
     * Physics simulation: the Spark MAX sim runs the same closed-loop
     * controller as the real hardware, its applied output drives the
     * elevator plant, and the resulting carriage motion is fed back into the
     * simulated encoder (in inches, matching the conversion factors).
     */
    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.02);

        leftMotorSim.iterate(
            Units.metersToInches(elevatorSim.getVelocityMetersPerSecond()),
            RobotController.getBatteryVoltage(),
            0.02);
    }
}
