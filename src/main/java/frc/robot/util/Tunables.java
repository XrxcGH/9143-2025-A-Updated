package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

import frc.robot.Constants.CorAlConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Live-tunable "magic numbers", backed by WPILib {@link Preferences}.
 *
 * Every value here can be edited from the dashboard WITHOUT changing or
 * redeploying code: Preferences live in NetworkTables under /Preferences,
 * Elastic's "Robot Preferences" widget (Testing tab) edits them in place, and
 * the roboRIO persists them to /home/lvuser/networktables.json - so an edit
 * survives reboots, power cycles, AND future code deploys (a deploy never
 * touches that file). The values in Constants are only the factory defaults,
 * seeded the first time the code runs (or after "Reset Tunables").
 *
 * What belongs here: empirically measured field/robot numbers that get
 * dialed in on the practice field (camera-read flush distances, branch
 * offsets, tracking gains, handoff timing) and the elevator's calibration
 * and Spark MAX tuning (travel ratio, gains, feedforward, profile), which
 * the Elevator re-applies to its controllers whenever the robot is disabled.
 * What does NOT belong here: the measured mechanism contact geometry (tuck
 * / low-box limits) - those are physical facts, not tuning knobs - and the
 * CorAl pivot's Phoenix gains, which are applied at boot and tuned live in
 * Phoenix Tuner X.
 *
 * Readers call the getters every time they need a value (a Preferences read
 * is one NetworkTables entry lookup - cheap), so edits take effect on the
 * next loop, on the next button press for values the Superstructure reads
 * at plan time, or on the next disable for the elevator controller values.
 * Every getter clamps to a sane range so a typo on the dashboard cannot
 * command something dangerous.
 */
public final class Tunables {
    private Tunables() {}

    // ------------------------------------------------------------------
    // Keys as shown in the Robot Preferences widget (grouped by prefix)
    // ------------------------------------------------------------------
    private static final String TELEOP_SPEED_SCALE = "Drive - Teleop Speed Scale (0-1)";
    private static final String ELEVATOR_TRAVEL_RATIO = "Elevator - Travel Ratio (measured / modeled)";
    private static final String ELEVATOR_ZERO_HEIGHT = "Elevator - Height At Hard Stop (in)";
    private static final String ELEVATOR_KP = "Elevator - kP (duty per in)";
    private static final String ELEVATOR_KS = "Elevator - kS (V)";
    private static final String ELEVATOR_KV_SCALE = "Elevator - kV Scale (x free-speed model)";
    private static final String ELEVATOR_KG = "Elevator - kG (V)";
    private static final String ELEVATOR_CRUISE_VELOCITY = "Elevator - Cruise Velocity (in/s)";
    private static final String ELEVATOR_MAX_ACCELERATION = "Elevator - Max Acceleration (in/s^2)";
    private static final String ELEVATOR_PROFILE_ERROR = "Elevator - Profile Error (in)";
    private static final String PIVOT_CRUISE_VELOCITY = "Pivot - Cruise Velocity (deg/s)";
    private static final String PIVOT_MAX_ACCELERATION = "Pivot - Acceleration (deg/s^2)";
    private static final String PIVOT_MAX_JERK = "Pivot - Jerk (deg/s^3)";
    private static final String CORAL_DETECT_DISTANCE = "CorAl - Coral Detect Distance (m)";
    private static final String CORAL_DETECT_HYSTERESIS = "CorAl - Coral Detect Hysteresis (m)";
    private static final String REEF_FLUSH_DISTANCE = "Vision - Reef Flush Distance (m)";
    private static final String STATION_FLUSH_DISTANCE = "Vision - Station Flush Distance (m)";
    private static final String L1_SCORE_DISTANCE = "Vision - L1 Score Distance (m)";
    private static final String REEF_BRANCH_OFFSET = "Vision - Reef Branch Offset (m)";
    private static final String TRACKING_DISTANCE_KP = "Vision - Tracking Distance kP (m/s per m)";
    private static final String TRACKING_ROTATION_KP = "Vision - Tracking Rotation kP (rad/s per deg)";
    private static final String DEFAULTS_VERSION_KEY = "Tunables - Defaults Version (do not edit)";

    /**
     * Version stamp of the factory defaults in Constants. Stored values
     * survive deploys, so changing a default in Constants does NOTHING on a
     * robot that already has the key stored - unless this number is bumped,
     * in which case init() overwrites EVERY tunable with the new defaults
     * once. Bump it when a default changes and must take effect on the
     * robot; leave it alone to preserve values tuned on the dashboard.
     *   1: initial tunables (Sept 2026)
     *   2: elevator calibration/gain tunables; elevator profile raised to
     *      16 in/s, 60 in/s^2; L4 arm arrival offset 0 s for that profile
     *   3: travel ratio back to 1.0 (gearing confirmed by 20/40 in tests);
     *      new "Height At Hard Stop" 0.875 in
     *   4: match-pace profiles - elevator 18 in/s, 120 in/s^2; new pivot
     *      profile tunables 240 deg/s, 480 deg/s^2, jerk 4800 deg/s^3
     *   5: elevator 20 in/s, 200 in/s^2 (gearing ceiling); pivot softened
     *      to 200 / 300 / 2000 for chain backlash; L4 arm arrival -0.6 s
     *      (was hitting the bar), L3 +0.8 s
     *   6: elevator regeared 15:1 - 50 in/s, 400 in/s^2, kP 0.1, kG 0.6;
     *      arm arrival offsets retired (the Superstructure now runs
     *      CAD-derived staged sequences gated on measured state)
     */
    private static final int DEFAULTS_VERSION = 6;

    /**
     * Seeds every key with its Constants default if it does not exist yet
     * (never overwrites a value the team has already tuned), or overwrites
     * all of them when {@link #DEFAULTS_VERSION} has been bumped since the
     * last boot. Call once at robot startup, BEFORE the subsystems are
     * constructed.
     */
    public static void init() {
        if (Preferences.getInt(DEFAULTS_VERSION_KEY, 0) != DEFAULTS_VERSION) {
            resetToDefaults();
            return;
        }
        Preferences.initDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.initDouble(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO);
        Preferences.initDouble(ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        Preferences.initDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
        Preferences.initDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
        Preferences.initDouble(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE);
        Preferences.initDouble(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG);
        Preferences.initDouble(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY);
        Preferences.initDouble(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        Preferences.initDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
        Preferences.initDouble(PIVOT_CRUISE_VELOCITY, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY);
        Preferences.initDouble(PIVOT_MAX_ACCELERATION, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        Preferences.initDouble(PIVOT_MAX_JERK, CorAlConstants.CORAL_PIVOT_MAX_JERK);
        Preferences.initDouble(CORAL_DETECT_DISTANCE, CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD);
        Preferences.initDouble(CORAL_DETECT_HYSTERESIS, CorAlConstants.GAME_PIECE_DETECTION_HYSTERESIS);
        Preferences.initDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.initDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.initDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.initDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.initDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.initDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
    }

    /**
     * Overwrites every tunable with its Constants default (the "Reset
     * Tunables" dashboard button, and a defaults-version bump at boot).
     */
    public static void resetToDefaults() {
        Preferences.setInt(DEFAULTS_VERSION_KEY, DEFAULTS_VERSION);
        Preferences.setDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.setDouble(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO);
        Preferences.setDouble(ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        Preferences.setDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
        Preferences.setDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
        Preferences.setDouble(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE);
        Preferences.setDouble(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG);
        Preferences.setDouble(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY);
        Preferences.setDouble(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        Preferences.setDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
        Preferences.setDouble(PIVOT_CRUISE_VELOCITY, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY);
        Preferences.setDouble(PIVOT_MAX_ACCELERATION, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        Preferences.setDouble(PIVOT_MAX_JERK, CorAlConstants.CORAL_PIVOT_MAX_JERK);
        Preferences.setDouble(CORAL_DETECT_DISTANCE, CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD);
        Preferences.setDouble(CORAL_DETECT_HYSTERESIS, CorAlConstants.GAME_PIECE_DETECTION_HYSTERESIS);
        Preferences.setDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.setDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.setDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.setDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.setDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.setDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
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
     * clamped to [0.05, 1.0] so a bad dashboard entry can neither disable
     * driving nor exceed the drivetrain's capability.
     */
    public static double teleopSpeedScale() {
        return clamped(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE, 0.05, 1.0);
    }

    // ------------------------------------------------------------------
    // Elevator calibration and Spark MAX tuning. The Elevator re-applies
    // these to both controllers the next time the robot is disabled; the
    // travel ratio additionally waits for the carriage to be at its base.
    // ------------------------------------------------------------------

    /**
     * Measured-over-modeled carriage travel: inches per motor rotation =
     * 0.244 x this. Clamped so a typo cannot scale the encoder by more than
     * ~3x in either direction (soft limits and presets are in inches).
     */
    public static double elevatorTravelRatio() {
        return clamped(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO, 0.33, 3.0);
    }

    /**
     * Height (preset frame: base-2x1 top to carriage-2x1 bottom) the
     * carriage sits at on its hard stop. The encoder is referenced to this
     * value at the hard stop and the reverse soft limit sits here. Clamped
     * to a few inches: anything larger is a measurement error.
     */
    public static double elevatorZeroHeight() {
        return clamped(ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_ZERO_HEIGHT, 0.0, 6.0);
    }

    /** Position loop proportional gain, duty cycle per inch of error. */
    public static double elevatorKp() {
        return clamped(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP, 0.0, 2.0);
    }

    /** Static friction feedforward, volts. */
    public static double elevatorKs() {
        return clamped(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS, 0.0, 3.0);
    }

    /** Multiplier on the NEO free-speed kV model (1.0 = pure back-EMF model). */
    public static double elevatorKvScale() {
        return clamped(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE, 0.0, 2.0);
    }

    /** Gravity feedforward, volts, applied at all times under position control. */
    public static double elevatorKg() {
        return clamped(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG, 0.0, 3.0);
    }

    /** MAXMotion cruise velocity, inches per second. */
    public static double elevatorCruiseVelocity() {
        return clamped(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY, 0.5, 40.0);
    }

    /** MAXMotion acceleration, inches per second squared. */
    public static double elevatorMaxAcceleration() {
        return clamped(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION, 1.0, 300.0);
    }

    /** Deviation from the MAXMotion profile (inches) that triggers a profile regeneration. */
    public static double elevatorProfileError() {
        return clamped(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR, 0.05, 5.0);
    }

    // ------------------------------------------------------------------
    // CorAl pivot Motion Magic profile (degrees). The CorAl re-applies
    // these to the TalonFX the next time the robot is disabled; the
    // Superstructure derives its handoff heights from the live values.
    // ------------------------------------------------------------------

    /** Motion Magic cruise velocity, deg/s (clamped below the ~550 deg/s free speed). */
    public static double pivotCruiseVelocity() {
        return clamped(PIVOT_CRUISE_VELOCITY, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY, 10.0, 500.0);
    }

    /** Motion Magic acceleration, deg/s^2. */
    public static double pivotMaxAcceleration() {
        return clamped(PIVOT_MAX_ACCELERATION, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION, 20.0, 3000.0);
    }

    /** Motion Magic jerk limit, deg/s^3 (0 disables the limit - a plain trapezoid). */
    public static double pivotMaxJerk() {
        return clamped(PIVOT_MAX_JERK, CorAlConstants.CORAL_PIVOT_MAX_JERK, 0.0, 50000.0);
    }

    // ------------------------------------------------------------------
    // CorAl coral detection (CANrange proximity). Re-applied to the sensor
    // by the CorAl the next time the robot is disabled.
    // ------------------------------------------------------------------

    /** CANrange proximity threshold, meters: a coral is "present" below it (minus the hysteresis). */
    public static double coralDetectDistance() {
        return clamped(CORAL_DETECT_DISTANCE, CorAlConstants.GAME_PIECE_DETECTION_THRESHOLD, 0.02, 0.5);
    }

    /** CANrange proximity hysteresis, meters, applied on both sides of the threshold. */
    public static double coralDetectHysteresis() {
        return clamped(CORAL_DETECT_HYSTERESIS, CorAlConstants.GAME_PIECE_DETECTION_HYSTERESIS, 0.0, 0.1);
    }

    // ------------------------------------------------------------------
    // Vision alignment goals (meters, camera-space)
    // ------------------------------------------------------------------

    /** Camera-read Z distance with the front bumpers flush on the reef base. */
    public static double reefFlushDistance() {
        return Preferences.getDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
    }

    /** Camera-read Z distance with the rear bumpers flush on the coral station wall. */
    public static double stationFlushDistance() {
        return Preferences.getDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
    }

    /** Standoff distance from a reef tag for L1 scoring. */
    public static double l1ScoreDistance() {
        return Preferences.getDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
    }

    /** Lateral offset from a reef tag center to a branch center. */
    public static double reefBranchOffset() {
        return Preferences.getDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
    }

    // ------------------------------------------------------------------
    // Vision tracking gains
    // ------------------------------------------------------------------

    /** m/s of drive command per meter of position error. */
    public static double trackingDistanceKp() {
        return Preferences.getDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
    }

    /** rad/s of rotation command per degree of angle error. */
    public static double trackingRotationKp() {
        return Preferences.getDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
    }

}
