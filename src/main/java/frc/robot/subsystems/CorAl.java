package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorAlConstants;
import frc.robot.util.Tunables;

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
public class CorAl extends SubsystemBase implements ArmAxis {
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
    // Both edges are debounced: a detection must persist to count as a
    // coral, and a loss must persist to count as gone, so a chattering
    // proximity bit cannot latch either way.
    private final Debouncer detectionDebouncer = new Debouncer(
        CorAlConstants.GAME_PIECE_DETECTION_CONFIRMATION_TIME, Debouncer.DebounceType.kBoth);
    private boolean gamePieceDetected = false;
    private boolean rawDetected = false; // Last raw (undebounced) verdict, for the dashboard
    // Latched side of the hysteresis band, so a reading inside the band
    // keeps whatever the last verdict was.
    private boolean detectLatch = false;

    // Applied CANrange detection tunables (re-applied while disabled; see periodic())
    private double appliedDetectThreshold;
    private double appliedDetectHysteresis;
    /** True = a piece reads CLOSER than the threshold; false = FARTHER. */
    private boolean appliedDetectWhenCloser;
    /** Minimum return strength for a measurement to count at all. */
    private double appliedMinSignalStrength;
    /** Master switch: false reports "nothing held" whatever the sensor says. */
    private boolean appliedDetectionEnabled;
    private double commandedIntakeSpeed = 0; // Last commanded roller speed (+ = coral intake direction)

    // Position Tracking
    private double currentTargetAngle = 0;         // Current target angle for position control (degrees)
    private boolean positionControlActive = false; // True while holding/moving to a closed-loop target
    private boolean manualControlActive = false;   // True while the operator stick is deflected

    // Through bore calibration: raw reading at the zero position
    private double throughBoreOffset = 0;

    // Landing correction (see PIVOT_LANDING_* in CorAlConstants)
    private final Timer landingStillTimer = new Timer();
    private int landingCorrections = 0;
    private double landingReferenceAngle = 0.0;

    // ------------------------------------------------------------------
    // The through bore is an ABSOLUTE encoder, but it used to be treated as
    // a relative one: every code start took "wherever the arm is now" as
    // zero. A restart with the arm raised (a brownout, a code crash, a
    // redeploy mid-practice) then shifted every Superstructure gate and
    // both soft limits by the arm's angle. The raw reading at the zero
    // position is now remembered on the roboRIO (Preferences; it survives
    // deploys) whenever the pivot is zeroed, and restored at the next start
    // once the encoder is reporting - so the arm knows its real angle
    // wherever it was when the code came up. A restored angle outside the
    // arm's travel means the encoder has moved on its shaft: it is refused
    // and this start's zero is kept, with a dashboard alert.
    // ------------------------------------------------------------------
    private static final String THROUGH_BORE_ZERO_KEY = "CorAl - Through Bore Zero (raw deg, set by zeroing)";
    private static final double RESTORE_MIN_ANGLE = -10.0;
    private static final double RESTORE_MAX_ANGLE = CorAlConstants.CORAL_PIVOT_MAX_ANGLE + 10.0;
    private boolean bootReferenceResolved = false;
    private boolean bootZeroRejected = false;
    private double bootRestoredAngle = 0.0;

    // ------------------------------------------------------------------
    // Live-tunable Motion Magic profile (Testing tab -> Tunables widget),
    // as last applied to the pivot TalonFX. See periodic().
    // ------------------------------------------------------------------
    private double appliedCruiseVelocity;  // deg/s
    private double appliedMaxAcceleration; // deg/s^2
    private double appliedMaxJerk;         // deg/s^3
    private double profileScale = 1.0;     // fraction of the tuned profile in effect (setProfileScale)
    private double appliedKg;              // V with the claw horizontal
    private double appliedBalanceAngle;    // deg where gravity does nothing
    private final Timer tunablePollTimer = new Timer();
    private static final double TUNABLE_POLL_SECONDS = 0.5;

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

        readProfileTunables();
        readDetectTunables();
        configurePivotMotor(pivotMotor);
        configureIntakeMotor(intakeMotor);
        configureCanRange(canRangeSensor);

        // Provisional zero: the current position. periodic() replaces it with
        // the stored absolute zero once the through bore is reporting.
        applyZeroHere();
        tunablePollTimer.start();

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
        applyZeroHere();
        if (isThroughBoreConnected()) {
            // Remember where zero is on the absolute encoder (see the note above)
            Preferences.setDouble(THROUGH_BORE_ZERO_KEY, throughBoreOffset);
            bootZeroRejected = false;
            bootRestoredAngle = 0.0;
        }
        bootReferenceResolved = true;
    }

    private void applyZeroHere() {
        throughBoreOffset = getRawThroughBoreAngle();
        pivotMotor.setPosition(0);
        currentTargetAngle = 0;
    }

    /**
     * Once per start, while disabled, as soon as the through bore reports:
     * restore the stored absolute zero (or, the first time ever, store this
     * start's).
     */
    private void resolveBootReference() {
        bootReferenceResolved = true;
        if (!Preferences.containsKey(THROUGH_BORE_ZERO_KEY)) {
            zeroEncoders(); // first start with this code: the arm is at its base, as it always had to be
            return;
        }
        double range = CorAlConstants.THROUGH_BORE_DEGREES_PER_ROTATION;
        double stored = Preferences.getDouble(THROUGH_BORE_ZERO_KEY, 0.0);
        double angle = getRawThroughBoreAngle() - stored;
        angle -= range * Math.round(angle / range);
        if (angle < RESTORE_MIN_ANGLE || angle > RESTORE_MAX_ANGLE) {
            bootZeroRejected = true; // keep this start's provisional zero
            applyZeroHere();
            return;
        }
        throughBoreOffset = stored;
        bootRestoredAngle = angle;
        pivotMotor.setPosition(angle / 360.0, 0);
        currentTargetAngle = angle;
    }

    /** True if the stored through bore zero put the arm outside its travel at this start and was refused. */
    public boolean isBootZeroRejected() {
        return bootZeroRejected;
    }

    /** The arm angle (deg) restored from the stored zero at this start; 0 if it started at its base or was just zeroed. */
    public double getBootRestoredAngle() {
        return bootRestoredAngle;
    }

    private void configurePivotMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimit = CorAlConstants.CORAL_PIVOT_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // Stator limit = torque cap: bounds how hard the 65:1 arm can ever
        // push if it meets a stop or a mis-tuned setpoint
        config.CurrentLimits.StatorCurrentLimit = CorAlConstants.CORAL_PIVOT_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Rotor-to-mechanism gearing: position feedback is in pivot rotations.
        // (Phoenix divides rotor rotations by this ratio; do NOT put a
        // rotations-to-degrees multiplier here.)
        config.Feedback.SensorToMechanismRatio = CorAlConstants.CORAL_PIVOT_GEAR_RATIO;

        // Closed-loop gains, profile feedforward and gravity (see slot0Config)
        config.Slot0 = slot0Config();

        // Motion Magic profile from the live tunables
        config.MotionMagic = motionMagicConfig();

        // Soft limits keep the arm inside its travel in every control mode
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CorAlConstants.CORAL_PIVOT_MAX_ANGLE / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CorAlConstants.CORAL_PIVOT_MIN_ANGLE / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    /**
     * Slot 0: closed-loop gains (error in mechanism rotations, output in
     * volts), the profile feedforward Motion Magic feeds its profiled
     * velocity / acceleration into, and gravity. 0 deg is not horizontal on
     * this arm, so Arm_Cosine is told where the balance point is: it outputs
     * kG x cos(position + offset) with offset = 0.25 rot - balance / 360
     * (Phoenix clamps the offset to +/-0.25 rot silently, hence the 1-179
     * deg clamp on the tunable). kG and the balance angle are live tunables;
     * kG ships at 0 until it has been measured (CORAL_PIVOT_kG).
     */
    private Slot0Configs slot0Config() {
        return new Slot0Configs()
            .withKP(CorAlConstants.CORAL_PIVOT_kP)
            .withKI(CorAlConstants.CORAL_PIVOT_kI)
            .withKD(CorAlConstants.CORAL_PIVOT_kD)
            .withKS(CorAlConstants.CORAL_PIVOT_kS)
            .withKV(CorAlConstants.CORAL_PIVOT_kV)
            .withKA(CorAlConstants.CORAL_PIVOT_kA)
            .withKG(appliedKg)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(0.25 - appliedBalanceAngle / 360.0);
    }

    /** Snapshots the profile and gravity tunables into the applied fields. */
    private void readProfileTunables() {
        appliedCruiseVelocity = Tunables.pivotCruiseVelocity();
        appliedMaxAcceleration = Tunables.pivotMaxAcceleration();
        appliedMaxJerk = Tunables.pivotMaxJerk();
        appliedKg = Tunables.pivotKg();
        appliedBalanceAngle = Tunables.pivotBalanceAngle();
    }

    /** True if any profile tunable differs from what is applied to the TalonFX. */
    private boolean profileTunablesChanged() {
        return Tunables.pivotCruiseVelocity() != appliedCruiseVelocity
            || Tunables.pivotMaxAcceleration() != appliedMaxAcceleration
            || Tunables.pivotMaxJerk() != appliedMaxJerk;
    }

    /** True if a gravity tunable differs from what is applied to the TalonFX. */
    private boolean gravityTunablesChanged() {
        return Tunables.pivotKg() != appliedKg || Tunables.pivotBalanceAngle() != appliedBalanceAngle;
    }

    /**
     * Motion Magic profile from the applied tunables. The tunables are in
     * degrees; Phoenix wants mechanism rotations. The jerk limit turns the
     * trapezoid into an S-curve: no step change in acceleration, so the arm
     * never snaps into or out of motion.
     */
    private MotionMagicConfigs motionMagicConfig() {
        // Time-scaling a profile by 1 / s keeps its shape: v x s, a x s^2, j x s^3.
        double s = profileScale;
        return new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(appliedCruiseVelocity * s / 360.0)
            .withMotionMagicAcceleration(appliedMaxAcceleration * s * s / 360.0)
            .withMotionMagicJerk(appliedMaxJerk * s * s * s / 360.0);
    }

    /**
     * Runs the pivot's profile at a fraction of its tuned speed, so a sweep
     * that would finish long before the carriage gets there can take its
     * time instead (the Superstructure paces the two mechanisms to travel
     * together). Only the Motion Magic group is sent, with a ZERO timeout -
     * fire and forget, this runs on the main loop - and only when the scale
     * actually changes. If the write were ever lost the arm would simply run
     * the previous profile: smoothness, not safety - the clearance clamps
     * read measured positions.
     */
    @Override
    public void setProfileScale(double scale) {
        scale = Math.min(Math.max(scale, 0.3), 1.0);
        if (Math.abs(scale - profileScale) < 0.01) {
            return;
        }
        profileScale = scale;
        pivotMotor.getConfigurator().apply(motionMagicConfig(), 0);
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
        config.ProximityParams = proximityConfig();
        // Narrow the beam: the full 27 deg cone picks up the claw's own
        // plates and rollers at oblique angles, and a coral fills a much
        // smaller cone at a few centimeters anyway.
        config.FovParams.FOVRangeX = CorAlConstants.GAME_PIECE_FOV_DEGREES;
        config.FovParams.FOVRangeY = CorAlConstants.GAME_PIECE_FOV_DEGREES;
        // Short-range mode at 100 Hz: the coral sits centimeters away, and
        // short range is the more robust mode at that distance.
        config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        sensor.getConfigurator().apply(config);
    }

    /** Snapshots the detection tunables into the applied fields. */
    private void readDetectTunables() {
        appliedDetectThreshold = Tunables.coralDetectDistance();
        appliedDetectHysteresis = Tunables.coralDetectHysteresis();
        appliedDetectWhenCloser = Tunables.coralDetectWhenCloser();
        appliedMinSignalStrength = Tunables.coralMinSignalStrength();
        appliedDetectionEnabled = Tunables.coralDetectionEnabled();
    }

    /** True if a detection tunable differs from what is applied to the sensor. */
    private boolean detectTunablesChanged() {
        return Tunables.coralDetectDistance() != appliedDetectThreshold
            || Tunables.coralDetectHysteresis() != appliedDetectHysteresis
            || Tunables.coralDetectWhenCloser() != appliedDetectWhenCloser
            || Tunables.coralMinSignalStrength() != appliedMinSignalStrength
            || Tunables.coralDetectionEnabled() != appliedDetectionEnabled;
    }

    /**
     * On-device proximity detection from the applied tunables: "detected"
     * below (threshold - hysteresis), "undetected" again only above
     * (threshold + hysteresis), and only while the return is strong enough
     * to be a valid measurement. The band is what stops the bit chattering
     * when the empty claw's own structure sits near the threshold.
     */
    private ProximityParamsConfigs proximityConfig() {
        return new ProximityParamsConfigs()
            .withProximityThreshold(appliedDetectThreshold)
            .withProximityHysteresis(appliedDetectHysteresis)
            .withMinSignalStrengthForValidMeasurement(appliedMinSignalStrength);
    }

    /**
     * Moves the pivot to the given angle (degrees) with Motion Magic and holds
     * it there. The motor sensor is re-seeded from the through bore encoder
     * before the move starts so the profile targets the true mechanism angle.
     */
    public void setPivotAngle(double targetAngle) {
        targetAngle = Math.min(Math.max(targetAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
                             CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        // Re-sending the target it is already holding would only re-seed
        // and restart the request for nothing.
        if (positionControlActive && targetAngle == currentTargetAngle) {
            return;
        }
        currentTargetAngle = targetAngle;
        positionControlActive = true;
        manualControlActive = false;
        landingCorrections = 0;
        landingStillTimer.restart();

        // Seed the motor sensor from the absolute encoder before starting the
        // move, so the profile targets the true mechanism angle with no
        // residual offset - but ONLY from rest. The Superstructure retargets
        // the arm mid-sweep (45 -> 100, 27.5 -> 20) at up to 300 deg/s, and
        // a seed taken there is stale by the DIO + CAN latency: 300 deg/s x
        // ~10 ms = 3 deg of step in the closed loop's feedback, in the
        // direction of travel, on top of the chain slack that separates the
        // two sensors under load. That step was the arm "jumping". Zero
        // timeout: the default setPosition overload BLOCKS the main loop
        // waiting for the device ack (up to 100 ms).
        if (isThroughBoreConnected()
                && Math.abs(getPivotVelocity()) <= CorAlConstants.PIVOT_RESEED_MAX_VELOCITY) {
            pivotMotor.setPosition(getThroughBoreAngle() / 360.0, 0);
        }

        pivotMotor.setControl(positionRequest.withPosition(targetAngle / 360.0));
    }

    /**
     * Landing correction: see PIVOT_LANDING_* in CorAlConstants. The rotor is
     * where it was told to be and the real arm, through the chain's slack, is
     * not - re-seed the rotor from the through bore and re-issue the same
     * target, so the closed loop drives the REAL arm onto it.
     */
    private void correctLanding() {
        if (!positionControlActive || !DriverStation.isEnabled() || !isThroughBoreConnected()) {
            landingStillTimer.restart();
            return;
        }
        double actual = getThroughBoreAngle();
        // "Still" means the rotor AND the real arm: while the carriage is
        // accelerating the arm swings in its chain slack with the rotor dead
        // still, and a correction taken off a swinging reading is a twitch.
        if (Math.abs(getPivotVelocity()) > CorAlConstants.PIVOT_LANDING_STILL_DEG_S
                || Math.abs(actual - landingReferenceAngle) > CorAlConstants.PIVOT_LANDING_STEADY_DEG) {
            landingStillTimer.restart();
            landingReferenceAngle = actual;
            return;
        }
        if (!landingStillTimer.hasElapsed(CorAlConstants.PIVOT_LANDING_STILL_SECONDS)
                || landingCorrections >= CorAlConstants.PIVOT_LANDING_MAX_CORRECTIONS) {
            return;
        }
        // Not against either end of travel: at 0 deg the arm is ON its hard
        // stop and at the forward soft limit the controller will not push
        // further, so a "correction" there only winds the rotor into the stop.
        if (currentTargetAngle < CorAlConstants.CORAL_PIVOT_MIN_ANGLE + CorAlConstants.PIVOT_LANDING_END_ZONE_DEG
                || currentTargetAngle > CorAlConstants.CORAL_PIVOT_MAX_ANGLE - CorAlConstants.PIVOT_LANDING_END_ZONE_DEG) {
            return;
        }
        double error = Math.abs(currentTargetAngle - actual);
        if (error <= CorAlConstants.PIVOT_LANDING_TOLERANCE_DEG || error > CorAlConstants.PIVOT_LANDING_MAX_DEG) {
            return;
        }
        landingCorrections++;
        landingStillTimer.restart();
        pivotMotor.setPosition(actual / 360.0, 0);
        pivotMotor.setControl(positionRequest.withPosition(currentTargetAngle / 360.0));
    }

    /** How many landing corrections the current target has had (0-PIVOT_LANDING_MAX_CORRECTIONS). */
    public int getLandingCorrections() {
        return landingCorrections;
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

    /** Pivot angular velocity in degrees per second (motor sensor). */
    public double getPivotVelocity() {
        return pivotMotor.getVelocity().getValueAsDouble() * 360.0;
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
     * The raw detection verdict, formed from the measured distance.
     *
     * The CANrange's own proximity bit is fixed to "distance below the
     * threshold". That is only the right question when a game piece is the
     * NEAREST thing the sensor can see; if the sensor looks across an empty
     * claw at structure a few centimetres away, an empty claw reads closer
     * than the threshold and the bit is stuck on - which is exactly the
     * "solid detected with nothing in the claw" symptom. So the verdict is
     * formed here instead, with the polarity as a tunable:
     *
     *   detect-when-closer  : a piece is CLOSER than the threshold
     *   detect-when-farther : a piece is FARTHER than the threshold (it
     *                         pushes the reflection away, or blocks a near
     *                         return the empty claw shows)
     *
     * A reading inside the hysteresis band around the threshold keeps the
     * previous verdict, a reading the sensor flags as unhealthy or too weak
     * counts as "nothing", and the caller debounces both edges. Set both the
     * threshold and the polarity from the dashboard: read
     * CorAl/CANrange Distance with the claw empty and with a coral held, put
     * the threshold halfway between, and set the polarity to whichever way
     * the coral moves the reading.
     */
    private boolean readDetection() {
        if (!appliedDetectionEnabled) {
            detectLatch = false;
            return false;
        }
        double strength = canRangeSensor.getSignalStrength().getValueAsDouble();
        // The strength gate is what separates a held coral from an empty
        // claw on this mechanism (65535 against 3000-4000). It latches: once
        // a return is strong enough to count, it keeps counting until the
        // strength drops well below the gate, so a coral held at an awkward
        // angle cannot blink out.
        double release = appliedMinSignalStrength * CorAlConstants.GAME_PIECE_STRENGTH_RELEASE_FRACTION;
        boolean strongEnough = detectLatch ? strength >= release : strength >= appliedMinSignalStrength;
        boolean valid = canRangeSensor.getMeasurementHealth().getValue() != MeasurementHealthValue.Bad
            && strongEnough;
        if (!valid) {
            detectLatch = false;
            return false;
        }
        double distance = canRangeSensor.getDistance().getValueAsDouble();
        double near = appliedDetectThreshold - appliedDetectHysteresis;
        double far = appliedDetectThreshold + appliedDetectHysteresis;
        if (distance <= near) {
            detectLatch = appliedDetectWhenCloser;
        } else if (distance >= far) {
            detectLatch = !appliedDetectWhenCloser;
        }
        // Inside the band: keep the last verdict.
        return detectLatch;
    }

    /**
     * Distance reading from the CANrange sensor in meters.
     */
    public double getCANRangeDistance() {
        return canRangeSensor.getDistance().getValueAsDouble();
    }

    /** CANrange return signal strength (unitless); below the configured minimum nothing can be detected. */
    public double getCANRangeSignalStrength() {
        return canRangeSensor.getSignalStrength().getValueAsDouble();
    }

    /** CANrange measurement health as reported by the sensor: Good, Limited, or Bad. */
    public String getCANRangeHealth() {
        return canRangeSensor.getMeasurementHealth().getValue().name();
    }

    /** The raw, undebounced verdict (after the validity check). */
    public boolean isCANRangeRawDetected() {
        return rawDetected;
    }

    /** True while "detected" means closer than the threshold, false while it means farther. */
    public boolean isDetectWhenCloser() {
        return appliedDetectWhenCloser;
    }

    /** Minimum return strength currently required for a measurement to count. */
    public double getMinSignalStrength() {
        return appliedMinSignalStrength;
    }

    /** False while game-piece detection is switched off (the intake then runs on the button alone). */
    public boolean isDetectionEnabled() {
        return appliedDetectionEnabled;
    }

    /** Proximity threshold (meters) currently applied to the CANrange. */
    public double getDetectThreshold() {
        return appliedDetectThreshold;
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

    /** Voltage the pivot motor is applying - what the kG measurement reads (see CORAL_PIVOT_kG). */
    public double getPivotVolts() {
        return pivotMotor.getMotorVoltage().getValueAsDouble();
    }

    /** Pivot motor applied duty cycle, -1 to 1. */
    public double getPivotOutput() {
        return pivotMotor.getDutyCycle().getValueAsDouble();
    }

    /** Intake motor applied duty cycle, -1 to 1. */
    public double getIntakeOutput() {
        return intakeMotor.getDutyCycle().getValueAsDouble();
    }

    // ------------------------------------------------------------------
    // Applied Motion Magic profile (the Superstructure planner derives its
    // handoff heights from these, so they must be the values in effect)
    // ------------------------------------------------------------------

    /** Cruise velocity (deg/s) currently applied to the pivot. */
    public double cruiseVelocity() {
        return appliedCruiseVelocity;
    }

    /** Acceleration (deg/s^2) currently applied to the pivot. */
    public double maxAcceleration() {
        return appliedMaxAcceleration * profileScale * profileScale;
    }

    /** Jerk limit (deg/s^3) currently applied to the pivot. */
    public double maxJerk() {
        return appliedMaxJerk;
    }

    @Override
    public void periodic() {
        // Game piece detection: see readDetection() - the verdict is formed
        // from the measured DISTANCE here rather than from the sensor's own
        // proximity bit, because that bit can only mean "closer than the
        // threshold" and which side of the threshold a coral puts the
        // reading on depends on where the sensor looks. Then debounced on
        // both edges. On the
        // confirmed rising edge (a coral just arrived), stop the rollers -
        // but only if they are running in the coral-intake (positive)
        // direction, so algae holding/ejecting is never interrupted by the
        // sensor.
        rawDetected = readDetection();
        boolean confirmed = detectionDebouncer.calculate(rawDetected);
        if (confirmed && !gamePieceDetected && commandedIntakeSpeed > 0) {
            stopIntake();
        }
        gamePieceDetected = confirmed;

        // Absolute zero: once per start, only while disabled (never move the
        // frame under a live setpoint), as soon as the encoder is reporting.
        if (!bootReferenceResolved && DriverStation.isDisabled() && isThroughBoreConnected()) {
            resolveBootReference();
        }

        correctLanding();

        // Keep the motor sensor honest against the absolute encoder, but only
        // while no closed-loop move is holding a target - re-seeding mid-move
        // shifts the reference frame and makes the arm land off target.
        if (!positionControlActive && !manualControlActive) {
            syncMotorToThroughBore();
        }

        // Re-apply an edited Motion Magic profile (Testing-tab tunables)
        // only while DISABLED - a config apply mid-move would stutter the
        // arm - polled twice a second. Only the MotionMagic group is sent,
        // so gains, limits, and the sensor ratio are untouched.
        if (DriverStation.isDisabled() && tunablePollTimer.advanceIfElapsed(TUNABLE_POLL_SECONDS)) {
            if (profileTunablesChanged() || gravityTunablesChanged()) {
                boolean gravity = gravityTunablesChanged();
                readProfileTunables();
                pivotMotor.getConfigurator().apply(motionMagicConfig());
                if (gravity) {
                    pivotMotor.getConfigurator().apply(slot0Config());
                }
            }
            // Coral detection threshold / hysteresis: only the proximity
            // group is sent to the CANrange.
            if (detectTunablesChanged()) {
                readDetectTunables();
                canRangeSensor.getConfigurator().apply(proximityConfig());
            }
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
