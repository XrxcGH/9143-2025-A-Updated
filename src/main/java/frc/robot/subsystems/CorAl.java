package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorAlConstants;

/**
 * The CorAl (Coral and Algae) subsystem controls the robot's game piece
 * manipulation mechanism: a pivot arm (Kraken X60) and intake rollers
 * (Kraken X60).
 *
 * Angle control design:
 * - The TalonFX rotor sensor is scaled by the pivot gear ratio, so the motor's
 *   position feedback is in mechanism rotations (degrees = rotations * 360).
 * - The REV Through Bore encoder is the absolute reference. The motor sensor
 *   is seeded from it at startup and re-synced whenever the arm is NOT in the
 *   middle of a closed-loop move (re-seeding mid-move shifts the reference
 *   frame and makes the arm land off target).
 * - Motion Magic runs the profile on the motor controller and holds the angle
 *   afterward; position control is never dropped when the target is reached.
 *
 * Game piece detection uses the CANrange's on-device proximity detection with
 * a rising-edge debounce to filter false positives.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class CorAl extends SubsystemBase {
    // Hardware Components
    private final TalonFX pivotMotor;             // Motor controlling the pivot mechanism
    private final TalonFX intakeMotor;            // Motor controlling the intake rollers
    private final DigitalInput throughBoreInput;  // Input for the through bore encoder
    private final DutyCycle throughBore;          // Duty cycle reader for the through bore encoder
    private final CANrange canRangeSensor;        // Distance sensor for game piece detection

    // Motor Control Objects (created once and reused)
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut percentRequest = new DutyCycleOut(0);

    // Game Piece Detection
    private final Debouncer detectionDebouncer = new Debouncer(
        CorAlConstants.GAME_PIECE_DETECTION_CONFIRMATION_TIME, Debouncer.DebounceType.kRising);
    private boolean gamePieceDetected = false;
    private double commandedIntakeSpeed = 0; // Last commanded roller speed (+ = coral intake direction)

    // Position Tracking
    private double currentTargetAngle = 0;         // Current target angle for position control (degrees)
    private boolean positionControlActive = false; // True while holding/moving to a closed-loop target
    private boolean manualControlActive = false;   // True while the operator stick is deflected

    // Through bore calibration: raw reading at the zero position
    private double throughBoreOffset = 0;

    // ------------------------------------------------------------------
    // Desktop simulation (only constructed when running off-robot). The
    // physics model exists purely so the mechanism moves in the sim GUI /
    // AdvantageScope; the values below affect simulation fidelity only.
    // Gravity is NOT simulated because the arm's zero is not horizontal
    // (matching kG = 0 in the real config) - enable both together once the
    // mounting orientation is verified.
    // In simulation the through bore reads disconnected, so getPivotAngle()
    // automatically falls back to the (simulated) motor sensor.
    // ------------------------------------------------------------------
    private static final double SIM_ARM_LENGTH_METERS = 0.4; // Estimate - affects sim only
    private static final double SIM_ARM_MASS_KG = 4.0;       // Estimate - affects sim only
    private SingleJointedArmSim armSim;

    public CorAl() {
        pivotMotor = new TalonFX(CorAlConstants.CORAL_PIVOT_MOTOR_ID);
        intakeMotor = new TalonFX(CorAlConstants.CORAL_INTAKE_MOTOR_ID);

        // REV Through Bore Encoder (absolute) on its DIO port
        throughBoreInput = new DigitalInput(CorAlConstants.THROUGH_BORE_DIO_PORT);
        throughBore = new DutyCycle(throughBoreInput);

        canRangeSensor = new CANrange(CorAlConstants.CANRANGE_SENSOR_ID);

        configurePivotMotor(pivotMotor);
        configureIntakeMotor(intakeMotor);
        configureCanRange(canRangeSensor);

        // Use the current position as the zero reference
        zeroEncoders();

        if (RobotBase.isSimulation()) {
            armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                CorAlConstants.CORAL_PIVOT_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(SIM_ARM_LENGTH_METERS, SIM_ARM_MASS_KG),
                SIM_ARM_LENGTH_METERS,
                Units.degreesToRadians(CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
                Units.degreesToRadians(CorAlConstants.CORAL_PIVOT_MAX_ANGLE),
                false, // No gravity - see class note above
                Units.degreesToRadians(CorAlConstants.CORAL_PIVOT_MIN_ANGLE));
        }
    }

    /**
     * Zeros both the motor encoder and through bore encoder so the current
     * position becomes the zero reference.
     */
    public void zeroEncoders() {
        throughBoreOffset = getRawThroughBoreAngle();
        pivotMotor.setPosition(0);
        currentTargetAngle = 0;
    }

    private void configurePivotMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_PIVOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Rotor-to-mechanism gearing: position feedback is in pivot rotations.
        // (Phoenix divides rotor rotations by this ratio; do NOT put a
        // rotations-to-degrees multiplier here.)
        config.Feedback.SensorToMechanismRatio = CorAlConstants.CORAL_PIVOT_GEAR_RATIO;

        // Closed-loop gains (error in mechanism rotations, output in volts)
        config.Slot0.kP = CorAlConstants.CORAL_PIVOT_kP;
        config.Slot0.kI = CorAlConstants.CORAL_PIVOT_kI;
        config.Slot0.kD = CorAlConstants.CORAL_PIVOT_kD;
        config.Slot0.kG = CorAlConstants.CORAL_PIVOT_kG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Motion Magic profile (constants are in degrees; Phoenix wants rotations)
        config.MotionMagic.MotionMagicCruiseVelocity = CorAlConstants.CORAL_PIVOT_MAX_VELOCITY / 360.0;
        config.MotionMagic.MotionMagicAcceleration = CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION / 360.0;

        // Soft limits keep the arm inside its travel in every control mode
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CorAlConstants.CORAL_PIVOT_MAX_ANGLE / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CorAlConstants.CORAL_PIVOT_MIN_ANGLE / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    private void configureIntakeMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = CorAlConstants.CORAL_INTAKE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_INTAKE_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    private void configureCanRange(CANrange sensor) {
        CANrangeConfiguration config = new CANrangeConfiguration();
        // On-device proximity detection: "detected" when something is closer
        // than the threshold
        config.ProximityParams.ProximityThreshold = CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD;
        sensor.getConfigurator().apply(config);
    }

    /**
     * Moves the pivot to the given angle (degrees) with Motion Magic and holds
     * it there. The motor sensor is re-seeded from the through bore encoder
     * before the move starts so the profile targets the true mechanism angle.
     */
    public void setPivotAngle(double targetAngle) {
        targetAngle = Math.min(Math.max(targetAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
                             CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        currentTargetAngle = targetAngle;
        positionControlActive = true;
        manualControlActive = false;

        // Seed the motor sensor from the absolute encoder before starting the
        // move (never mid-move; see periodic()), so the profile targets the
        // true mechanism angle with no residual offset. Zero timeout: the
        // default setPosition overload BLOCKS the main loop waiting for the
        // device ack (up to 100 ms); fire-and-forget still applies the value.
        if (isThroughBoreConnected()) {
            pivotMotor.setPosition(getThroughBoreAngle() / 360.0, 0);
        }

        pivotMotor.setControl(positionRequest.withPosition(targetAngle / 360.0));
    }

    /**
     * Sets the motor sensor to the through bore angle if they disagree.
     * Non-blocking (zero timeout) because this runs on the main loop -
     * from periodic() it could otherwise stall the whole robot for a CAN
     * config round trip on every loop while the discrepancy persists.
     */
    private void syncMotorToThroughBore() {
        if (!isThroughBoreConnected()) {
            return;
        }
        double throughBoreAngle = getThroughBoreAngle();
        if (Math.abs(throughBoreAngle - getMotorAngle()) > CorAlConstants.THROUGH_BORE_ALLOWED_DISCREPANCY) {
            pivotMotor.setPosition(throughBoreAngle / 360.0, 0);
        }
    }

    /**
     * Resets the pivot encoder to calibrate the system.
     */
    public void resetPivotEncoder() {
        zeroEncoders();
    }

    /**
     * Sets the intake roller speed. Positive = coral intake direction.
     *
     * @param speed The speed value (-1.0 to 1.0)
     */
    public void setIntakeSpeed(double speed) {
        commandedIntakeSpeed = speed;
        intakeMotor.setControl(percentRequest.withOutput(speed));
    }

    /**
     * Stops the intake roller motor.
     */
    public void stopIntake() {
        setIntakeSpeed(0);
    }

    /**
     * Manual pivot control from the operator stick. Takes the raw stick value;
     * deadband and speed limiting are applied here.
     *
     * While the stick is deflected the arm runs open loop (overriding any
     * active position control). When the stick is released the arm captures
     * and holds its current angle under closed-loop control instead of going
     * limp and falling.
     */
    public void manualPivotControl(double stickInput) {
        if (Math.abs(stickInput) >= CorAlConstants.CORAL_MANUAL_CONTROL_DEADBAND) {
            positionControlActive = false;
            manualControlActive = true;
            double speed = stickInput * CorAlConstants.CORAL_MANUAL_SPEED_LIMIT;
            pivotMotor.setControl(percentRequest.withOutput(speed));
        } else if (manualControlActive) {
            // Stick just released - hold the current angle
            manualControlActive = false;
            setPivotAngle(getPivotAngle());
        }
        // Otherwise: position control (if active) keeps holding on the motor
        // controller; nothing to do.
    }

    /**
     * Stops the pivot motor and drops any closed-loop target.
     */
    public void stopPivot() {
        positionControlActive = false;
        manualControlActive = false;
        pivotMotor.setControl(percentRequest.withOutput(0));
    }

    /**
     * Raw through bore reading in degrees, without the zero offset.
     * The encoder's duty cycle spans one full revolution.
     */
    public double getRawThroughBoreAngle() {
        return throughBore.getOutput() * CorAlConstants.THROUGH_BORE_DEGREES_PER_ROTATION;
    }

    /**
     * Calibrated through bore angle in degrees (0 at the zeroed position).
     * The offset difference is wrapped to the shortest path within the
     * encoder's range, which handles the reading wrapping around without any
     * stateful unwrap tracking (safe to call from multiple readers).
     */
    public double getThroughBoreAngle() {
        if (!isThroughBoreConnected()) {
            // Fall back to the motor sensor if the through bore is disconnected
            return getMotorAngle();
        }

        double range = CorAlConstants.THROUGH_BORE_DEGREES_PER_ROTATION;
        double delta = getRawThroughBoreAngle() - throughBoreOffset;
        // Wrap into [-range/2, range/2)
        delta -= range * Math.round(delta / range);
        return delta;
    }

    /** Pivot angle in degrees according to the motor's (seeded) sensor. */
    public double getMotorAngle() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    /**
     * The current pivot angle in degrees; through bore is the primary source.
     */
    public double getPivotAngle() {
        return getThroughBoreAngle();
    }

    /**
     * Checks if the pivot is at the target angle within allowed error.
     */
    public boolean isAtTargetAngle() {
        return Math.abs(getPivotAngle() - currentTargetAngle) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }

    /**
     * Checks if the motor sensor agrees with the through bore encoder.
     */
    public boolean isMotorFeedbackValid() {
        if (!isThroughBoreConnected()) {
            return true; // Assume valid if the Through Bore Encoder is disconnected
        }
        double discrepancy = Math.abs(getMotorAngle() - getThroughBoreAngle());
        return discrepancy <= CorAlConstants.THROUGH_BORE_ALLOWED_DISCREPANCY;
    }

    /**
     * Checks if the through bore encoder is connected.
     */
    public boolean isThroughBoreConnected() {
        return throughBore.getFrequency() > 0;
    }

    /**
     * Distance reading from the CANrange sensor in meters.
     */
    public double getCANRangeDistance() {
        return canRangeSensor.getDistance().getValueAsDouble();
    }

    /**
     * Whether a game piece has been detected (debounced). Updated in periodic().
     */
    public boolean isGamePieceDetected() {
        return gamePieceDetected;
    }

    // ------------------------------------------------------------------
    // Diagnostic getters (used by the Dashboard)
    // ------------------------------------------------------------------

    /** The angle in degrees the closed loop is targeting. */
    public double getTargetAngle() {
        return currentTargetAngle;
    }

    /** Pivot motor supply current in amps. */
    public double getPivotCurrent() {
        return pivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    /** Intake motor supply current in amps. */
    public double getIntakeCurrent() {
        return intakeMotor.getSupplyCurrent().getValueAsDouble();
    }

    /** Pivot motor applied duty cycle, -1 to 1. */
    public double getPivotOutput() {
        return pivotMotor.getDutyCycle().getValueAsDouble();
    }

    /** Intake motor applied duty cycle, -1 to 1. */
    public double getIntakeOutput() {
        return intakeMotor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Debounced game piece detection using the CANrange's on-device
        // proximity bit. On the rising edge (a coral just arrived), stop the
        // rollers - but only if they are running in the coral-intake
        // (positive) direction, so algae holding/ejecting is never
        // interrupted by the sensor.
        boolean confirmed = detectionDebouncer.calculate(canRangeSensor.getIsDetected().getValue());
        if (confirmed && !gamePieceDetected && commandedIntakeSpeed > 0) {
            stopIntake();
        }
        gamePieceDetected = confirmed;

        // Keep the motor sensor honest against the absolute encoder, but only
        // while no closed-loop move is holding a target - re-seeding mid-move
        // shifts the reference frame and makes the arm land off target.
        if (!positionControlActive && !manualControlActive) {
            syncMotorToThroughBore();
        }
    }

    /**
     * Physics simulation: the TalonFX sim runs the same Motion Magic
     * profile as the real hardware, its output voltage drives the arm
     * plant, and the resulting motion is fed back into the simulated rotor
     * (rotor rotations = mechanism rotations x gear ratio, matching
     * SensorToMechanismRatio).
     */
    @Override
    public void simulationPeriodic() {
        var pivotSimState = pivotMotor.getSimState();
        pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        armSim.setInputVoltage(pivotSimState.getMotorVoltage());
        armSim.update(0.02);

        pivotSimState.setRawRotorPosition(
            Units.radiansToRotations(armSim.getAngleRads()) * CorAlConstants.CORAL_PIVOT_GEAR_RATIO);
        pivotSimState.setRotorVelocity(
            Units.radiansToRotations(armSim.getVelocityRadPerSec()) * CorAlConstants.CORAL_PIVOT_GEAR_RATIO);
    }
}
