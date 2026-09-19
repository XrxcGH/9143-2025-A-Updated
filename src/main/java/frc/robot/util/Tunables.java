package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

import frc.robot.Constants.CorAlConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TunablesConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Live-tunable "magic numbers", backed by WPILib {@link Preferences}.
 *
 * Every value here can be edited from the dashboard without changing or
 * redeploying code: Preferences live in NetworkTables under /Preferences,
 * Elastic's "Robot Preferences" widget (Testing tab) edits them in place, and
 * the roboRIO persists them to /home/lvuser/networktables.json - so an edit
 * survives reboots, power cycles, and future code deploys (a deploy never
 * touches that file). The values in Constants are only the factory defaults,
 * seeded the first time the code runs (or after "Reset Tunables").
 *
 * What belongs here: empirically measured field/robot numbers that get
 * dialed in on the practice field (camera-read flush distances, branch
 * offset, tracking gains, the teleop speed scale), the elevator's
 * calibration and Spark MAX tuning (travel ratio, hard-stop height, gains,
 * feedforward, profile), the pivot's Motion Magic profile and gravity
 * feedforward, and the CANrange coral-detection thresholds. The Elevator
 * and the CorAl re-apply theirs to the hardware whenever the robot is
 * disabled.
 * What does not belong here: the measured mechanism contact geometry (tuck
 * / low-box limits) - those are physical facts, not tuning knobs - and the
 * CorAl pivot's Phoenix closed-loop gains, which are applied at boot and
 * tuned live in Phoenix Tuner X.
 *
 * Readers call the getters every time they need a value (a Preferences read
 * is one NetworkTables entry lookup - cheap), so edits take effect on the
 * next loop, on the next button press for values the Superstructure reads
 * at plan time, or on the next disable for the values held on a motor
 * controller or sensor (elevator, pivot, CANrange). Every numeric getter
 * clamps to its range in Constants.TunablesConstants (the two tracking gains
 * to a multiple of their default) so a typo on the dashboard cannot command
 * something dangerous. The defaults version is there too; this class holds
 * only the keys and the logic.
 */
public final class Tunables {
    private Tunables() {}

    // ------------------------------------------------------------------
    // Keys as shown in the Robot Preferences widget (grouped by prefix).
    // They stay here, not in Constants: renaming a key loses its stored value.
    // ------------------------------------------------------------------
    private static final String TELEOP_SPEED_SCALE = "Drive - Teleop Speed Scale (0-1)";
    private static final String ELEVATOR_TRAVEL_RATIO = "Elevator - Travel Ratio (measured / modeled)";
    private static final String ELEVATOR_ZERO_HEIGHT = "Elevator - Height At Hard Stop (in)";
    private static final String ELEVATOR_KP = "Elevator - kP (duty per in)";
    private static final String ELEVATOR_KS = "Elevator - kS (V)";
    private static final String ELEVATOR_KV_SCALE = "Elevator - kV Scale (x free-speed model)";
    private static final String ELEVATOR_KA = "Elevator - kA (V per in/s^2)";
    private static final String ELEVATOR_KG = "Elevator - kG (V)";
    private static final String ELEVATOR_CRUISE_VELOCITY = "Elevator - Cruise Velocity (in/s)";
    private static final String ELEVATOR_MAX_ACCELERATION = "Elevator - Max Acceleration (in/s^2)";
    private static final String ELEVATOR_PROFILE_ERROR = "Elevator - Profile Error (in)";
    private static final String PIVOT_CRUISE_VELOCITY = "Pivot - Cruise Velocity (deg/s)";
    private static final String PIVOT_MAX_ACCELERATION = "Pivot - Acceleration (deg/s^2)";
    private static final String PIVOT_MAX_JERK = "Pivot - Jerk (deg/s^3)";
    private static final String PIVOT_KG = "Pivot - kG (V, claw horizontal)";
    private static final String PIVOT_BALANCE_ANGLE = "Pivot - Balance Angle (deg)";
    private static final String CORAL_DETECT_DISTANCE = "CorAl - Coral Detect Distance (m)";
    private static final String CORAL_DETECT_HYSTERESIS = "CorAl - Coral Detect Hysteresis (m)";
    private static final String CORAL_DETECT_WHEN_CLOSER = "CorAl - Coral Detect When Closer (1) Or Farther (0)";
    private static final String CORAL_MIN_SIGNAL_STRENGTH = "CorAl - Coral Min Signal Strength";
    private static final String CORAL_DETECTION_ENABLED = "CorAl - Coral Detection Enabled (1) Or Off (0)";
    private static final String REEF_FLUSH_DISTANCE = "Vision - Reef Flush Distance (m)";
    private static final String STATION_FLUSH_DISTANCE = "Vision - Station Flush Distance (m)";
    private static final String L1_SCORE_DISTANCE = "Vision - L1 Score Distance (m)";
    private static final String BARGE_SCORE_DISTANCE = "Vision - Barge Score Distance (m)";
    private static final String PROCESSOR_DISTANCE = "Vision - Processor Distance (m)";
    private static final String REEF_BRANCH_OFFSET = "Vision - Reef Branch Offset (m)";
    private static final String TRACKING_DISTANCE_KP = "Vision - Tracking Distance kP (m/s per m)";
    private static final String TRACKING_ROTATION_KP = "Vision - Tracking Rotation kP (rad/s per deg)";
    private static final String DEFAULTS_VERSION_KEY = "Tunables - Defaults Version (do not edit)";

    /**
     * Seeds every key with its Constants default if it does not exist yet
     * (never overwrites a value the team has already tuned). When
     * TunablesConstants.DEFAULTS_VERSION has been bumped since the last boot
     * it instead runs the targeted migration for the stored version or, if
     * there is none, overwrites every key. Call once at robot startup,
     * before the subsystems are constructed.
     */
    public static void init() {
        // 14 -> 15 is a targeted migration: only the keys whose defaults
        // changed are overwritten (elevator kP 0.4, kS 0, profile error
        // 1.0 in - the values that stop the carriage ringing as it settles
        // on a setpoint), so everything else the team has tuned on the
        // dashboard (vision distances, speed scale, ...) survives. A bump
        // otherwise resets every tunable; add a block like this one whenever
        // only a few defaults move. (14 and 15 name stored defaults versions,
        // so they stay here: they are identifiers, not settings.)
        if (Preferences.getInt(DEFAULTS_VERSION_KEY, 0) == 14) {
            Preferences.setDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
            Preferences.setDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
            Preferences.setDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
            Preferences.setInt(DEFAULTS_VERSION_KEY, 15);
        }
        if (Preferences.getInt(DEFAULTS_VERSION_KEY, 0) != TunablesConstants.DEFAULTS_VERSION) {
            resetToDefaults();
            return;
        }
        Preferences.initDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.initDouble(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO);
        Preferences.initDouble(ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        Preferences.initDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
        Preferences.initDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
        Preferences.initDouble(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE);
        Preferences.initDouble(ELEVATOR_KA, ElevatorConstants.ELEVATOR_kA);
        Preferences.initDouble(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG);
        Preferences.initDouble(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY);
        Preferences.initDouble(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        Preferences.initDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
        Preferences.initDouble(PIVOT_CRUISE_VELOCITY, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY);
        Preferences.initDouble(PIVOT_MAX_ACCELERATION, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        Preferences.initDouble(PIVOT_MAX_JERK, CorAlConstants.CORAL_PIVOT_MAX_JERK);
        Preferences.initDouble(PIVOT_KG, CorAlConstants.CORAL_PIVOT_kG);
        Preferences.initDouble(PIVOT_BALANCE_ANGLE, CorAlConstants.CORAL_PIVOT_BALANCE_ANGLE_DEG);
        Preferences.initDouble(CORAL_DETECT_DISTANCE, CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD);
        Preferences.initDouble(CORAL_DETECT_HYSTERESIS, CorAlConstants.GAME_PIECE_DETECTION_HYSTERESIS);
        Preferences.initDouble(CORAL_DETECT_WHEN_CLOSER,
            CorAlConstants.GAME_PIECE_DETECT_WHEN_CLOSER ? 1.0 : 0.0);
        Preferences.initDouble(CORAL_MIN_SIGNAL_STRENGTH, CorAlConstants.GAME_PIECE_MIN_SIGNAL_STRENGTH);
        Preferences.initDouble(CORAL_DETECTION_ENABLED,
            CorAlConstants.GAME_PIECE_DETECTION_ENABLED ? 1.0 : 0.0);
        Preferences.initDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.initDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.initDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.initDouble(BARGE_SCORE_DISTANCE, VisionConstants.BARGE_SCORE_DISTANCE);
        Preferences.initDouble(PROCESSOR_DISTANCE, VisionConstants.PROCESSOR_DISTANCE);
        Preferences.initDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.initDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.initDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
    }

    /**
     * Overwrites every tunable with its Constants default and stamps the
     * current defaults version (the "Reset Tunables" dashboard button, and
     * a defaults-version bump at boot that has no targeted migration).
     */
    public static void resetToDefaults() {
        Preferences.setInt(DEFAULTS_VERSION_KEY, TunablesConstants.DEFAULTS_VERSION);
        Preferences.setDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.setDouble(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO);
        Preferences.setDouble(ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        Preferences.setDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
        Preferences.setDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
        Preferences.setDouble(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE);
        Preferences.setDouble(ELEVATOR_KA, ElevatorConstants.ELEVATOR_kA);
        Preferences.setDouble(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG);
        Preferences.setDouble(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY);
        Preferences.setDouble(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        Preferences.setDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
        Preferences.setDouble(PIVOT_CRUISE_VELOCITY, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY);
        Preferences.setDouble(PIVOT_MAX_ACCELERATION, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        Preferences.setDouble(PIVOT_MAX_JERK, CorAlConstants.CORAL_PIVOT_MAX_JERK);
        Preferences.setDouble(PIVOT_KG, CorAlConstants.CORAL_PIVOT_kG);
        Preferences.setDouble(PIVOT_BALANCE_ANGLE, CorAlConstants.CORAL_PIVOT_BALANCE_ANGLE_DEG);
        Preferences.setDouble(CORAL_DETECT_DISTANCE, CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD);
        Preferences.setDouble(CORAL_DETECT_HYSTERESIS, CorAlConstants.GAME_PIECE_DETECTION_HYSTERESIS);
        Preferences.setDouble(CORAL_DETECT_WHEN_CLOSER,
            CorAlConstants.GAME_PIECE_DETECT_WHEN_CLOSER ? 1.0 : 0.0);
        Preferences.setDouble(CORAL_MIN_SIGNAL_STRENGTH, CorAlConstants.GAME_PIECE_MIN_SIGNAL_STRENGTH);
        Preferences.setDouble(CORAL_DETECTION_ENABLED,
            CorAlConstants.GAME_PIECE_DETECTION_ENABLED ? 1.0 : 0.0);
        Preferences.setDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.setDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.setDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.setDouble(BARGE_SCORE_DISTANCE, VisionConstants.BARGE_SCORE_DISTANCE);
        Preferences.setDouble(PROCESSOR_DISTANCE, VisionConstants.PROCESSOR_DISTANCE);
        Preferences.setDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.setDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.setDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
    }

    /**
     * Like {@link #clamped} but on the magnitude: the vision goal readouts
     * are signed (a station tag reads negative), and a pasted negative
     * value must not invert a goal.
     */
    private static double clampedMagnitude(String key, double defaultValue, double min, double max) {
        double value = Math.abs(Preferences.getDouble(key, defaultValue));
        return Math.max(min, Math.min(max, value));
    }

    /** Reads a key, falling back to its default, and clamps the result to [min, max]. */
    private static double clamped(String key, double defaultValue, double min, double max) {
        double value = Preferences.getDouble(key, defaultValue);
        return Math.max(min, Math.min(max, value));
    }

    // ------------------------------------------------------------------
    // Driving
    // ------------------------------------------------------------------

    /**
     * Fraction of theoretical top speed / rotation rate at full stick,
     * clamped so a bad dashboard entry can neither disable driving nor
     * exceed the drivetrain's capability.
     */
    public static double teleopSpeedScale() {
        return clamped(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE,
            TunablesConstants.TELEOP_SPEED_SCALE_MIN, TunablesConstants.TELEOP_SPEED_SCALE_MAX);
    }

    // ------------------------------------------------------------------
    // Elevator calibration and Spark MAX tuning. The Elevator re-applies
    // these to both controllers the next time the robot is disabled; the
    // travel ratio additionally waits for the carriage to be at its base.
    // ------------------------------------------------------------------

    /**
     * Measured-over-modeled carriage travel: inches per motor rotation =
     * ~0.733 (ELEVATOR_MODELED_INCHES_PER_ROTATION, the 15:1 gearing model)
     * x this. Clamped so a typo cannot scale the encoder by more than
     * ~3x in either direction (soft limits and presets are in inches).
     */
    public static double elevatorTravelRatio() {
        return clamped(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO,
            TunablesConstants.ELEVATOR_TRAVEL_RATIO_MIN, TunablesConstants.ELEVATOR_TRAVEL_RATIO_MAX);
    }

    /**
     * Height (preset frame: base-2x1 top to carriage-2x1 bottom) the
     * carriage sits at on its hard stop. The encoder is referenced to this
     * value at the hard stop and the reverse soft limit sits here. Clamped
     * to a few inches: anything larger is a measurement error.
     */
    public static double elevatorZeroHeight() {
        return clamped(ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_ZERO_HEIGHT,
            TunablesConstants.ELEVATOR_ZERO_HEIGHT_MIN, TunablesConstants.ELEVATOR_ZERO_HEIGHT_MAX);
    }

    /** Position loop proportional gain, duty cycle per inch of error. */
    public static double elevatorKp() {
        return clamped(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP,
            TunablesConstants.ELEVATOR_KP_MIN, TunablesConstants.ELEVATOR_KP_MAX);
    }

    /** Static friction feedforward, volts. */
    public static double elevatorKs() {
        return clamped(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS,
            TunablesConstants.ELEVATOR_KS_MIN, TunablesConstants.ELEVATOR_KS_MAX);
    }

    /** Multiplier on the NEO free-speed kV model (1.0 = pure back-EMF model). */
    public static double elevatorKvScale() {
        return clamped(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE,
            TunablesConstants.ELEVATOR_KV_SCALE_MIN, TunablesConstants.ELEVATOR_KV_SCALE_MAX);
    }

    /**
     * Acceleration feedforward, volts per in/s^2 of profile acceleration.
     * Clamped to about three times the CAD model: more than that would
     * mean the carriage weighs far more than the CAD says.
     */
    public static double elevatorKa() {
        return clamped(ELEVATOR_KA, ElevatorConstants.ELEVATOR_kA,
            TunablesConstants.ELEVATOR_KA_MIN, TunablesConstants.ELEVATOR_KA_MAX);
    }

    /**
     * Gravity feedforward, volts, applied at all times under position
     * control. Set it from the "Elevator/kG From Cruise" dashboard readout.
     */
    public static double elevatorKg() {
        return clamped(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG,
            TunablesConstants.ELEVATOR_KG_MIN, TunablesConstants.ELEVATOR_KG_MAX);
    }

    /** MAXMotion cruise velocity, inches per second. */
    public static double elevatorCruiseVelocity() {
        return clamped(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY,
            TunablesConstants.ELEVATOR_CRUISE_VELOCITY_MIN, TunablesConstants.ELEVATOR_CRUISE_VELOCITY_MAX);
    }

    /** MAXMotion acceleration, inches per second squared. */
    public static double elevatorMaxAcceleration() {
        return clamped(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION,
            TunablesConstants.ELEVATOR_MAX_ACCELERATION_MIN, TunablesConstants.ELEVATOR_MAX_ACCELERATION_MAX);
    }

    /** Deviation from the MAXMotion profile (inches) that triggers a profile regeneration. */
    public static double elevatorProfileError() {
        return clamped(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR,
            TunablesConstants.ELEVATOR_PROFILE_ERROR_MIN, TunablesConstants.ELEVATOR_PROFILE_ERROR_MAX);
    }

    // ------------------------------------------------------------------
    // CorAl pivot Motion Magic profile (degrees). The CorAl re-applies
    // these to the TalonFX the next time the robot is disabled; the
    // Superstructure derives its handoff heights from the live values.
    // ------------------------------------------------------------------

    /** Motion Magic cruise velocity, deg/s (clamped below the ~550 deg/s free speed). */
    public static double pivotCruiseVelocity() {
        return clamped(PIVOT_CRUISE_VELOCITY, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY,
            TunablesConstants.PIVOT_CRUISE_VELOCITY_MIN, TunablesConstants.PIVOT_CRUISE_VELOCITY_MAX);
    }

    /** Motion Magic acceleration, deg/s^2. */
    public static double pivotMaxAcceleration() {
        return clamped(PIVOT_MAX_ACCELERATION, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION,
            TunablesConstants.PIVOT_MAX_ACCELERATION_MIN, TunablesConstants.PIVOT_MAX_ACCELERATION_MAX);
    }

    /**
     * Pivot gravity feedforward, volts with the claw horizontal (the most
     * gravity the arm ever sees). Not yet measured: the default is 0 (no
     * gravity feedforward) - see the procedure at CORAL_PIVOT_kG in
     * Constants. Clamped to about three times the CAD estimate (~0.3 V).
     */
    public static double pivotKg() {
        return clamped(PIVOT_KG, CorAlConstants.CORAL_PIVOT_kG,
            TunablesConstants.PIVOT_KG_MIN, TunablesConstants.PIVOT_KG_MAX);
    }

    /**
     * Arm angle (deg) at which gravity does nothing: the claw balanced
     * straight up. The default is an estimate, to be measured together
     * with the pivot kG.
     */
    public static double pivotBalanceAngle() {
        return clamped(PIVOT_BALANCE_ANGLE, CorAlConstants.CORAL_PIVOT_BALANCE_ANGLE_DEG,
            TunablesConstants.PIVOT_BALANCE_ANGLE_MIN, TunablesConstants.PIVOT_BALANCE_ANGLE_MAX);
    }

    /** Motion Magic jerk limit, deg/s^3 (0 disables the limit - a plain trapezoid). */
    public static double pivotMaxJerk() {
        return clamped(PIVOT_MAX_JERK, CorAlConstants.CORAL_PIVOT_MAX_JERK,
            TunablesConstants.PIVOT_MAX_JERK_MIN, TunablesConstants.PIVOT_MAX_JERK_MAX);
    }

    // ------------------------------------------------------------------
    // CorAl coral detection (CANrange proximity). Re-applied to the sensor
    // by the CorAl the next time the robot is disabled.
    // ------------------------------------------------------------------

    /** CANrange proximity threshold, meters: a coral is "present" below it (minus the hysteresis). */
    public static double coralDetectDistance() {
        return clamped(CORAL_DETECT_DISTANCE, CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD,
            TunablesConstants.CORAL_DETECT_DISTANCE_MIN, TunablesConstants.CORAL_DETECT_DISTANCE_MAX);
    }

    /**
     * Which way a coral moves the CANrange reading: 1 = a piece reads
     * closer than the detect distance, 0 = a piece reads farther. Set it
     * from the dashboard by reading CorAl/CANrange Distance with the claw
     * empty and with a coral held.
     */
    public static boolean coralDetectWhenCloser() {
        return Preferences.getDouble(CORAL_DETECT_WHEN_CLOSER,
            CorAlConstants.GAME_PIECE_DETECT_WHEN_CLOSER ? 1.0 : 0.0) >= 0.5;
    }

    /**
     * Minimum CANrange return strength for a measurement to count at all.
     * When the sensor looks into open air with nothing held, the return is
     * weak and this alone separates "holding" from "empty" (CTRE's tuning
     * guide, scenario 1) - read CorAl/CANrange Signal Strength empty and
     * holding a coral and split the two.
     */
    public static double coralMinSignalStrength() {
        return clamped(CORAL_MIN_SIGNAL_STRENGTH, CorAlConstants.GAME_PIECE_MIN_SIGNAL_STRENGTH,
            TunablesConstants.CORAL_MIN_SIGNAL_STRENGTH_MIN, TunablesConstants.CORAL_MIN_SIGNAL_STRENGTH_MAX);
    }

    /**
     * Master switch for game-piece detection. Set it to 0 while the
     * thresholds are still being found: the intake then runs on the
     * operator's button alone instead of skipping itself because the sensor
     * claims a coral is already held.
     */
    public static boolean coralDetectionEnabled() {
        return Preferences.getDouble(CORAL_DETECTION_ENABLED,
            CorAlConstants.GAME_PIECE_DETECTION_ENABLED ? 1.0 : 0.0) >= 0.5;
    }

    /** CANrange proximity hysteresis, meters, applied on both sides of the threshold. */
    public static double coralDetectHysteresis() {
        return clamped(CORAL_DETECT_HYSTERESIS, CorAlConstants.GAME_PIECE_DETECTION_HYSTERESIS,
            TunablesConstants.CORAL_DETECT_HYSTERESIS_MIN, TunablesConstants.CORAL_DETECT_HYSTERESIS_MAX);
    }

    // ------------------------------------------------------------------
    // Vision alignment goals (meters, robot frame: where the tag sits relative to the robot center)
    // ------------------------------------------------------------------

    /** Forward distance from the robot center to a reef tag with the front bumpers flush on the reef base. */
    public static double reefFlushDistance() {
        return clampedMagnitude(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE,
            TunablesConstants.REEF_FLUSH_DISTANCE_MIN, TunablesConstants.REEF_FLUSH_DISTANCE_MAX);
    }

    /** Distance from the robot center back to a coral-station tag with the rear bumpers flush (used as a negative forward goal). */
    public static double stationFlushDistance() {
        return clampedMagnitude(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE,
            TunablesConstants.STATION_FLUSH_DISTANCE_MIN, TunablesConstants.STATION_FLUSH_DISTANCE_MAX);
    }

    /** Standoff distance from a reef tag for L1 scoring. */
    public static double l1ScoreDistance() {
        return clampedMagnitude(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE,
            TunablesConstants.L1_SCORE_DISTANCE_MIN, TunablesConstants.L1_SCORE_DISTANCE_MAX);
    }

    /** Distance from the robot center to a barge tag at the net-shot position (centered, square). */
    public static double bargeScoreDistance() {
        return clampedMagnitude(BARGE_SCORE_DISTANCE, VisionConstants.BARGE_SCORE_DISTANCE,
            TunablesConstants.BARGE_SCORE_DISTANCE_MIN, TunablesConstants.BARGE_SCORE_DISTANCE_MAX);
    }

    /** Distance from the robot center to a processor tag with the front bumper just off the wall. */
    public static double processorDistance() {
        return clampedMagnitude(PROCESSOR_DISTANCE, VisionConstants.PROCESSOR_DISTANCE,
            TunablesConstants.PROCESSOR_DISTANCE_MIN, TunablesConstants.PROCESSOR_DISTANCE_MAX);
    }

    /** Lateral offset from a reef tag center to a branch center. */
    public static double reefBranchOffset() {
        return clampedMagnitude(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET,
            TunablesConstants.REEF_BRANCH_OFFSET_MIN, TunablesConstants.REEF_BRANCH_OFFSET_MAX);
    }

    // ------------------------------------------------------------------
    // Vision tracking gains
    // ------------------------------------------------------------------

    /** m/s of drive command per meter of position error, clamped to 0 - TRACKING_KP_MAX_MULTIPLE x the default. */
    public static double trackingDistanceKp() {
        return clamped(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP,
            TunablesConstants.TRACKING_DISTANCE_KP_MIN,
            TunablesConstants.TRACKING_KP_MAX_MULTIPLE * VisionConstants.TrackingGains.DISTANCE_kP);
    }

    /** rad/s of rotation command per degree of angle error, clamped to 0 - TRACKING_KP_MAX_MULTIPLE x the default. */
    public static double trackingRotationKp() {
        return clamped(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP,
            TunablesConstants.TRACKING_ROTATION_KP_MIN,
            TunablesConstants.TRACKING_KP_MAX_MULTIPLE * VisionConstants.TrackingGains.ROTATION_kP);
    }

}
