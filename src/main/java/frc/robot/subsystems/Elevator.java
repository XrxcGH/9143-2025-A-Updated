package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Tunables;

/**
 * Elevator subsystem driven by two NEO brushless motors on Spark MAX
 * controllers, each through a 15:1 MAXPlanetary reduction (5:1 x 3:1
 * cartridges) and a 90 deg gearbox to the 22T sprocket shaft.
 *
 * Control architecture:
 *  - The left Spark MAX is the leader; the right is configured as a hardware
 *    follower, so both sides always receive identical output and cannot
 *    fight each other.
 *  - The encoder conversion factors scale the NEO's integrated encoder so
 *    every position is in inches and every velocity in inches per second.
 *    The inches-per-rotation figure is the gearing model times the
 *    "Elevator - Travel Ratio" tunable (confirmed 1.0 on the robot).
 *  - Heights are in the preset frame: top of the base-stage 2x1 to the
 *    bottom of the carriage 2x1. On its hard stop the carriage sits above
 *    that reference by "Elevator - Height At Hard Stop" (1.000 in, the
 *    middle-stage tube), so the encoder is zeroed to that value, not to 0,
 *    and the reverse soft limit sits there (README: "Calibrating the
 *    elevator height").
 *  - Height moves use MAXMotion (trapezoidal profiling on the controller)
 *    with on-controller kS/kV/kA/kG feedforward, so the carriage tracks
 *    smoothly and holds its height at rest and when the operator releases
 *    the stick.
 *  - Soft limits on the controller bound travel in every control mode, and
 *    voltage compensation keeps response consistent as the battery sags.
 *
 * Live tuning: the travel ratio, gains, feedforward, and profile limits are
 * Preferences-backed tunables (Testing tab). {@link #periodic()} re-applies
 * an edit to both controllers the next time the robot is disabled, so the
 * elevator is calibrated and tuned without a redeploy or the REV Hardware
 * Client. A travel-ratio or hard-stop-height edit additionally waits for
 * the carriage to be at its base, because it rescales or re-references the
 * encoder; the encoders are then re-referenced there.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class Elevator extends SubsystemBase implements CarriageAxis {

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
    // The pace of the setpoint in effect (one Spark MAX closed-loop slot each)
    private Pace currentPace = Pace.FULL;
    private static final ClosedLoopSlot[] PACE_SLOTS = {
        ClosedLoopSlot.kSlot0, ClosedLoopSlot.kSlot1, ClosedLoopSlot.kSlot2, ClosedLoopSlot.kSlot3};

    // ------------------------------------------------------------------
    // Live-tunable configuration (Testing tab -> Tunables widget), as last
    // applied to the controllers. See periodic().
    // ------------------------------------------------------------------
    private double appliedTravelRatio;
    /** Height (preset frame, inches) of the carriage on its hard stop; the encoder is zeroed to this. */
    private double appliedZeroHeight;
    private double appliedKp;
    private double appliedKs;
    private double appliedKvScale;
    private double appliedKa;
    private double appliedKg;
    private double appliedCruiseVelocity;
    private double appliedMaxAcceleration;
    private double appliedProfileError;
    /** True while an edited travel ratio / hard-stop height is waiting for the carriage to be at its base. */
    private boolean travelRatioChangePending = false;
    /** New setpoints sent to the controller since boot (a diagnostic: it must not climb while the carriage is holding still). */
    private int setpointCount = 0;
    /** Filtered applied volts at steady cruise, up and down (NaN until seen); see sampleCruiseVolts(). */
    private double cruiseVoltsUp = Double.NaN;
    private double cruiseVoltsDown = Double.NaN;
    /** Paces the tunable poll so the Preferences reads do not run every loop. */
    private final Timer tunablePollTimer = new Timer();
    private static final double TUNABLE_POLL_SECONDS = 0.5;

    // ------------------------------------------------------------------
    // Desktop simulation (only constructed when running off-robot). The
    // physics model exists purely so the mechanism moves in the sim GUI /
    // AdvantageScope; the values below affect simulation fidelity only.
    // ------------------------------------------------------------------
    private static final double SIM_CARRIAGE_MASS_KG = 6.0; // Estimate - affects sim only
    private SparkMaxSim leftMotorSim;
    private ElevatorSim elevatorSim;

    public Elevator() {
        leftMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);

        appliedTravelRatio = Tunables.elevatorTravelRatio();
        appliedZeroHeight = Tunables.elevatorZeroHeight();
        readGainTunables();
        configureMotors(ResetMode.kResetSafeParameters);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        // All closed-loop commands go to the leader; the follower mirrors it
        closedLoopController = leftMotor.getClosedLoopController();

        // Reference the encoders on initialization (the carriage must be on
        // its hard stop at power-on)
        resetEncoders();
        tunablePollTimer.start();

        if (RobotBase.isSimulation()) {
            leftMotorSim = new SparkMaxSim(leftMotor, DCMotor.getNEO(1));
            // Effective drum radius derived from the same conversion the real
            // controller uses (carriage inches per motor rotation x gear
            // ratio = inches per drum rotation), so the sim cannot drift
            // from the calibrated mechanism scaling.
            double drumRadiusMeters = Units.inchesToMeters(
                inchesPerRotation() * ElevatorConstants.ELEVATOR_GEAR_RATIO) / (2.0 * Math.PI);
            elevatorSim = new ElevatorSim(
                DCMotor.getNEO(2),
                ElevatorConstants.ELEVATOR_GEAR_RATIO,
                SIM_CARRIAGE_MASS_KG,
                drumRadiusMeters,
                Units.inchesToMeters(appliedZeroHeight),
                Units.inchesToMeters(ElevatorConstants.ELEVATOR_MAX_POSITION),
                true, // Simulate gravity so kG/holding behavior is visible
                Units.inchesToMeters(appliedZeroHeight));
        }
    }

    /** Snapshots the gain / feedforward / profile tunables into the applied fields. */
    private void readGainTunables() {
        appliedKp = Tunables.elevatorKp();
        appliedKs = Tunables.elevatorKs();
        appliedKvScale = Tunables.elevatorKvScale();
        appliedKa = Tunables.elevatorKa();
        appliedKg = Tunables.elevatorKg();
        appliedCruiseVelocity = Tunables.elevatorCruiseVelocity();
        appliedMaxAcceleration = Tunables.elevatorMaxAcceleration();
        appliedProfileError = Tunables.elevatorProfileError();
    }

    /** True if any gain / feedforward / profile tunable differs from what is applied. */
    private boolean gainTunablesChanged() {
        return Tunables.elevatorKp() != appliedKp
            || Tunables.elevatorKs() != appliedKs
            || Tunables.elevatorKvScale() != appliedKvScale
            || Tunables.elevatorKa() != appliedKa
            || Tunables.elevatorKg() != appliedKg
            || Tunables.elevatorCruiseVelocity() != appliedCruiseVelocity
            || Tunables.elevatorMaxAcceleration() != appliedMaxAcceleration
            || Tunables.elevatorProfileError() != appliedProfileError;
    }

    /**
     * Builds and applies the leader and follower configurations from the
     * applied tunables. Parameters are persisted to flash so a brownout or
     * power cycle cannot silently revert the controllers to factory
     * defaults mid-match.
     *
     * @param resetMode kResetSafeParameters at boot (start from a known
     *     state); kNoResetSafeParameters for a live re-apply, which only
     *     touches the parameters in the config.
     */
    private void configureMotors(ResetMode resetMode) {
        double inchesPerRotation = inchesPerRotation();
        // Velocity feedforward is the NEO back-EMF model in the current
        // encoder units, so it follows the travel ratio automatically.
        double kV = ElevatorConstants.modelKv(inchesPerRotation) * appliedKvScale;

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
            .positionConversionFactor(inchesPerRotation)
            .velocityConversionFactor(inchesPerRotation / 60.0)
            // Faster velocity measurement than the 32 ms x 8 default: the
            // controller restarts a MAXMotion profile from the measured
            // state, and ~130 ms of velocity lag there restarts it at a
            // speed the carriage no longer has.
            .uvwMeasurementPeriod(ElevatorConstants.ELEVATOR_VELOCITY_PERIOD_MS)
            .uvwAverageDepth(ElevatorConstants.ELEVATOR_VELOCITY_AVG_DEPTH);

        // The closed loop runs on the NEO's integrated encoder. Error units
        // are inches after the conversion factors above; PID output is duty
        // cycle.
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // One closed-loop slot per Pace: identical gains and feedforward, and
        // MAXMotion limits scaled by the pace. The Superstructure picks the
        // pace with each setpoint so the carriage travels with the arm
        // instead of racing it to a clearance limit and braking there.
        //
        // On-controller feedforward (volts): static friction, velocity, and
        // gravity compensation - the same kS/kV/kG model as WPILib's
        // ElevatorFeedforward, evaluated by the Spark MAX every cycle so the
        // carriage tracks the MAXMotion profile and holds height at rest.
        // kA is applied in MAXMotion modes only (REVLib FeedForwardConfig)
        // and is what keeps the carriage on the profile through the
        // acceleration and deceleration ramps instead of buying that force
        // with position error.
        //
        // MAXMotion profile parameters (inches, inches per second). The
        // profile error is how far the carriage may stray from the profile
        // before MAXMotion regenerates it from the current state - it is
        // not a settling tolerance.
        for (Pace pace : Pace.values()) {
            ClosedLoopSlot slot = PACE_SLOTS[pace.ordinal()];
            leaderConfig.closedLoop
                .pid(appliedKp, ElevatorConstants.ELEVATOR_kI, ElevatorConstants.ELEVATOR_kD, slot)
                .outputRange(-1, 1, slot);
            leaderConfig.closedLoop.feedForward
                .svag(appliedKs, kV, appliedKa, appliedKg, slot);
            leaderConfig.closedLoop.maxMotion
                .cruiseVelocity(appliedCruiseVelocity * pace.scale, slot)
                .maxAcceleration(appliedMaxAcceleration * pace.scale, slot)
                .allowedProfileError(appliedProfileError, slot);
        }

        // Soft limits (inches, preset frame) bound travel in every control
        // mode. The reverse limit is the hard-stop height, since that is
        // where the encoder reads with the carriage at the bottom.
        leaderConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.ELEVATOR_MAX_POSITION)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(appliedZeroHeight)
            .reverseSoftLimitEnabled(true);

        // --- Follower configuration ---
        // Same current limit, brake mode, voltage compensation, and encoder
        // scaling as the leader, but it follows the leader's output and
        // carries no soft limits of its own. REV documents only that a
        // follower mirrors the leader's voltage output; whether it still
        // enforces its own soft limits is undocumented, and the follower
        // spins opposite the leader (mirrored mounting) so its encoder
        // counts negative as the carriage rises - leader limits copied onto
        // it would put its reverse limit at zero in force for the entire
        // climb, turning the follower into a brake the leader must drag
        // (slow, stuttering, current-limited climbs). The leader's soft
        // limits bound the mechanism on their own.
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.apply(leaderConfig);
        followerConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
        followerConfig.follow(ElevatorConstants.ELEVATOR_LEFT_ID,
            ElevatorConstants.ELEVATOR_RIGHT_OPPOSES_LEFT);

        // Leader only, and therefore after the follower copy above (a config
        // keeps the shortest period ever set on it, and apply() copies it).
        // Height and velocity frames at 10 ms instead of the 20 ms default:
        // every Superstructure gate and clamp reads these on the roboRIO, and
        // at 40 in/s a 20 ms-old height is 0.8 in out of date.
        leaderConfig.signals
            .primaryEncoderPositionPeriodMs(ElevatorConstants.ELEVATOR_STATUS_PERIOD_MS)
            .primaryEncoderVelocityPeriodMs(ElevatorConstants.ELEVATOR_STATUS_PERIOD_MS);

        leftMotor.configure(leaderConfig, resetMode, PersistMode.kPersistParameters);
        rightMotor.configure(followerConfig, resetMode, PersistMode.kPersistParameters);
    }

    /**
     * Measures kG in a way that does not depend on friction or on the
     * configured gains: the applied voltage at steady cruise going up is
     * kG + kV x v + friction, going down it is kG - kV x v - friction, so
     * their mean is kG. (The hold voltage cannot tell you: inside the static
     * friction band it only echoes the gains already configured.) Samples
     * are taken only under position control within 5 % of the cruise
     * velocity, and low-pass filtered per direction. Run a long move each
     * way - Testing tab, arm at RAISE - and read Elevator/kG From Cruise.
     */
    private void sampleCruiseVolts() {
        if (!positionControlEnabled || appliedCruiseVelocity <= 0) {
            return;
        }
        double velocity = getVelocity();
        if (Math.abs(Math.abs(velocity) - appliedCruiseVelocity) > 0.05 * appliedCruiseVelocity) {
            return;
        }
        double volts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        if (velocity > 0) {
            cruiseVoltsUp = Double.isNaN(cruiseVoltsUp) ? volts : cruiseVoltsUp + 0.1 * (volts - cruiseVoltsUp);
        } else {
            cruiseVoltsDown = Double.isNaN(cruiseVoltsDown) ? volts : cruiseVoltsDown + 0.1 * (volts - cruiseVoltsDown);
        }
    }

    /** New closed-loop setpoints sent since boot. While the carriage is holding still this must not change. */
    public int getSetpointCount() {
        return setpointCount;
    }

    /** Filtered applied volts at steady cruise going up (NaN until an upward cruise has been seen). */
    public double getCruiseVoltsUp() {
        return cruiseVoltsUp;
    }

    /** Filtered applied volts at steady cruise going down (NaN until a downward cruise has been seen). */
    public double getCruiseVoltsDown() {
        return cruiseVoltsDown;
    }

    /** The kG (volts) the two cruise voltages imply: their mean. NaN until both directions have been seen. */
    public double getKgFromCruise() {
        return (cruiseVoltsUp + cruiseVoltsDown) / 2.0;
    }

    /**
     * Samples the cruise voltage every loop, then re-applies edited tunables
     * to the controllers. The re-apply runs only while disabled (a
     * reconfigure mid-move would stutter the mechanism), polled twice a
     * second. Gains, feedforward, and profile limits apply right away. The
     * calibration values (travel ratio, hard-stop height) rescale or
     * re-reference the encoder, so they are applied only with the carriage
     * at its base, where the encoders are then re-referenced against the
     * hard stop; until then they wait and the Dashboard shows an alert.
     */
    @Override
    public void periodic() {
        sampleCruiseVolts();
        if (!DriverStation.isDisabled() || !tunablePollTimer.advanceIfElapsed(TUNABLE_POLL_SECONDS)) {
            return;
        }

        boolean reconfigure = false;
        boolean calibrationApplied = false;

        double ratio = Tunables.elevatorTravelRatio();
        double zeroHeight = Tunables.elevatorZeroHeight();
        if (ratio != appliedTravelRatio || zeroHeight != appliedZeroHeight) {
            if (isAtBase()) {
                appliedTravelRatio = ratio;
                appliedZeroHeight = zeroHeight;
                travelRatioChangePending = false;
                calibrationApplied = true;
                reconfigure = true;
            } else {
                travelRatioChangePending = true;
            }
        } else {
            travelRatioChangePending = false;
        }

        if (gainTunablesChanged()) {
            readGainTunables();
            reconfigure = true;
        }

        if (reconfigure) {
            configureMotors(ResetMode.kNoResetSafeParameters);
            if (calibrationApplied) {
                // The carriage is on its hard stop: reference the new
                // scale / hard-stop height exactly there.
                resetEncoders();
            }
        }
    }

    /**
     * Commands the elevator to a height in inches (preset frame, clamped to
     * the travel limits) using MAXMotion at the given pace. The Spark MAX
     * runs the profile and holds the position afterward; gravity/friction
     * compensation comes from the configured kS/kV/kA/kG. A setpoint
     * identical to the one already running is not re-sent.
     */
    @Override
    public void setPosition(double targetPosition, Pace pace) {
        // Clamp target position within safe limits (a preset of 0 means "as
        // low as it goes", which is the hard-stop height)
        targetPosition = Math.min(Math.max(targetPosition, appliedZeroHeight),
            ElevatorConstants.ELEVATOR_MAX_POSITION);

        // MAXMotion regenerates its profile from the measured state whenever
        // a setpoint arrives, so never re-send the one it is already running.
        if (positionControlEnabled && targetPosition == currentTargetPosition && pace == currentPace) {
            return;
        }

        currentTargetPosition = targetPosition;
        currentPace = pace;
        setpointCount++;
        positionControlEnabled = true;
        manualModeEnabled = false;

        // The controller latches the setpoint; no need to re-send every loop
        closedLoopController.setSetpoint(targetPosition, ControlType.kMAXMotionPositionControl,
            PACE_SLOTS[pace.ordinal()]);
    }

    /**
     * Holds the current height under closed-loop control. Used when the
     * robot is enabled, so a carriage that was left raised does not sink
     * under gravity before the first command arrives.
     */
    public void holdCurrentPosition() {
        // Already holding a setpoint right here: keep it. A new setpoint at
        // the measured height is a zero-length MAXMotion profile - at best
        // pointless, and it moves the hold by the resting error.
        if (positionControlEnabled
                && Math.abs(currentTargetPosition - getCurrentPosition()) <= ElevatorConstants.ELEVATOR_HOLD_KEEP_WINDOW
                && Math.abs(getVelocity()) <= 1.0) {
            return;
        }
        setPosition(getCurrentPosition());
    }

    /**
     * References both encoders to the hard-stop height; only do this with
     * the carriage resting on its hard stop. (Named "reset" on the
     * dashboard / controller because that is what the operator does; the
     * value written is the calibrated hard-stop height, not zero.)
     */
    public void resetEncoders() {
        leftEncoder.setPosition(appliedZeroHeight);
        rightEncoder.setPosition(appliedZeroHeight);
    }

    /**
     * Open-loop voltage control from the operator stick: the gravity
     * feedforward plus the stick scaled to ELEVATOR_MANUAL_MAX_VOLTS. The
     * leader's soft limits stop travel at either end of the elevator's range.
     *
     * @param speed stick value, -1 to 1 (positive = up); deadbanded here
     */
    public void manualControl(double speed) {
        // Manual input overrides position control
        positionControlEnabled = false;
        manualModeEnabled = true;

        // Apply deadband and limits
        if (Math.abs(speed) < ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        // Gravity plus the stick, in volts. Plain duty cycle has no gravity
        // term: with about a volt needed to hold, a small up-stick
        // would let the carriage sink while the operator pushed up, and full
        // down would run far faster than full up. With kG added, the stick
        // commands the same speed in either direction.
        speed = Math.min(Math.max(speed, -1), 1);
        leftMotor.setVoltage(appliedKg + speed * ElevatorConstants.ELEVATOR_MANUAL_MAX_VOLTS);
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
     * True when both sides report the same amount of travel. Compared by
     * magnitude because the follower spins opposite the leader and its
     * encoder counts the other way. A persistent mismatch means slippage
     * or a mechanical problem; the Dashboard raises an alert.
     */
    public boolean sidesInSync() {
        double leftTravel = Math.abs(leftEncoder.getPosition() - appliedZeroHeight);
        double rightTravel = Math.abs(rightEncoder.getPosition() - appliedZeroHeight);
        return Math.abs(leftTravel - rightTravel) <= ElevatorConstants.ELEVATOR_SIDE_SYNC_TOLERANCE;
    }

    /** True when the carriage is (by the encoder) resting on its hard stop. */
    public boolean isAtBase() {
        return Math.abs(getCurrentPosition() - appliedZeroHeight) <= ElevatorConstants.ELEVATOR_AT_BASE_TOLERANCE;
    }

    /** Follower (right) encoder position in inches, sign as the follower reports it, for diagnostics. */
    public double getFollowerPosition() {
        return rightEncoder.getPosition();
    }

    /**
     * True when the encoder reads meaningfully below the hard-stop height:
     * it was referenced with the carriage raised and the carriage has since
     * dropped to the hard stop, so every commanded height would land that
     * much high. The Dashboard raises an alert; fix by re-zeroing at the
     * hard stop.
     */
    public boolean readsBelowZero() {
        return getCurrentPosition() < appliedZeroHeight + ElevatorConstants.ELEVATOR_BELOW_ZERO_ALERT;
    }

    public boolean isInManualMode() {
        return manualModeEnabled;
    }

    // ------------------------------------------------------------------
    // Applied configuration (used by the Superstructure planner and Dashboard)
    // ------------------------------------------------------------------

    /** Travel ratio (measured / modeled) currently applied to the encoder scaling. */
    public double travelRatio() {
        return appliedTravelRatio;
    }

    /** Height (preset frame, inches) the encoder reads with the carriage on its hard stop. */
    public double zeroHeight() {
        return appliedZeroHeight;
    }

    /** Carriage travel (inches) per motor rotation currently applied to the controllers. */
    public double inchesPerRotation() {
        return ElevatorConstants.ELEVATOR_MODELED_INCHES_PER_ROTATION * appliedTravelRatio;
    }

    /** Static friction feedforward (volts) currently applied; the controller adds it at rest too. */
    public double staticFeedforward() {
        return appliedKs;
    }

    /** Acceleration feedforward (volts per in/s^2) currently applied. */
    public double accelerationFeedforward() {
        return appliedKa;
    }

    /** MAXMotion cruise velocity (in/s) currently applied - the planner derives handoffs from it. */
    public double cruiseVelocity() {
        return appliedCruiseVelocity;
    }

    /** MAXMotion acceleration (in/s^2) in effect at the current pace - the planner sizes braking distances from it. */
    @Override
    public double maxAcceleration() {
        return appliedMaxAcceleration * currentPace.scale;
    }

    @Override
    public double maxAcceleration(Pace pace) {
        return appliedMaxAcceleration * pace.scale;
    }

    /** True while an edited travel ratio or hard-stop height is waiting for the carriage to be at its base. */
    public boolean isTravelRatioChangePending() {
        return travelRatioChangePending;
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
