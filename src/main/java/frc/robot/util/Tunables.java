package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

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
    private static final String ELEVATOR_KP = "Elevator - kP (duty per in)";
    private static final String ELEVATOR_KS = "Elevator - kS (V)";
    private static final String ELEVATOR_KV_SCALE = "Elevator - kV Scale (x free-speed model)";
    private static final String ELEVATOR_KG = "Elevator - kG (V)";
    private static final String ELEVATOR_CRUISE_VELOCITY = "Elevator - Cruise Velocity (in/s)";
    private static final String ELEVATOR_MAX_ACCELERATION = "Elevator - Max Acceleration (in/s^2)";
    private static final String ELEVATOR_PROFILE_ERROR = "Elevator - Profile Error (in)";
    private static final String REEF_FLUSH_DISTANCE = "Vision - Reef Flush Distance (m)";
    private static final String STATION_FLUSH_DISTANCE = "Vision - Station Flush Distance (m)";
    private static final String L1_SCORE_DISTANCE = "Vision - L1 Score Distance (m)";
    private static final String REEF_BRANCH_OFFSET = "Vision - Reef Branch Offset (m)";
    private static final String TRACKING_DISTANCE_KP = "Vision - Tracking Distance kP (m/s per m)";
    private static final String TRACKING_ROTATION_KP = "Vision - Tracking Rotation kP (rad/s per deg)";
    private static final String L3_ARM_ARRIVAL_OFFSET = "Superstructure - L3 Arm Arrival Offset (s)";
    private static final String L4_ARM_ARRIVAL_OFFSET = "Superstructure - L4 Arm Arrival Offset (s)";

    /**
     * Seeds every key with its Constants default if it does not exist yet
     * (never overwrites a value the team has already tuned). Call once at
     * robot startup, BEFORE the subsystems are constructed.
     */
    public static void init() {
        Preferences.initDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.initDouble(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO);
        Preferences.initDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
        Preferences.initDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
        Preferences.initDouble(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE);
        Preferences.initDouble(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG);
        Preferences.initDouble(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY);
        Preferences.initDouble(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        Preferences.initDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
        Preferences.initDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.initDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.initDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.initDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.initDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.initDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
        Preferences.initDouble(L3_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L3_ARM_ARRIVAL_OFFSET_SECONDS);
        Preferences.initDouble(L4_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L4_ARM_ARRIVAL_OFFSET_SECONDS);
    }

    /** Overwrites every tunable with its Constants default (the "Reset Tunables" dashboard button). */
    public static void resetToDefaults() {
        Preferences.setDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.setDouble(ELEVATOR_TRAVEL_RATIO, ElevatorConstants.ELEVATOR_MEASURED_TRAVEL_RATIO);
        Preferences.setDouble(ELEVATOR_KP, ElevatorConstants.ELEVATOR_kP);
        Preferences.setDouble(ELEVATOR_KS, ElevatorConstants.ELEVATOR_kS);
        Preferences.setDouble(ELEVATOR_KV_SCALE, ElevatorConstants.ELEVATOR_kV_SCALE);
        Preferences.setDouble(ELEVATOR_KG, ElevatorConstants.ELEVATOR_kG);
        Preferences.setDouble(ELEVATOR_CRUISE_VELOCITY, ElevatorConstants.ELEVATOR_MAX_VELOCITY);
        Preferences.setDouble(ELEVATOR_MAX_ACCELERATION, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        Preferences.setDouble(ELEVATOR_PROFILE_ERROR, ElevatorConstants.ELEVATOR_ALLOWED_PROFILE_ERROR);
        Preferences.setDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.setDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.setDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.setDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.setDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.setDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
        Preferences.setDouble(L3_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L3_ARM_ARRIVAL_OFFSET_SECONDS);
        Preferences.setDouble(L4_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L4_ARM_ARRIVAL_OFFSET_SECONDS);
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

    // ------------------------------------------------------------------
    // Superstructure handoff timing (seconds) - read at plan time. The
    // handoff HEIGHTS are derived from these plus the motion profiles, so
    // they stay in sync with any elevator/pivot retune (see
    // Superstructure.handoffHeight).
    // ------------------------------------------------------------------

    /** Seconds after the elevator settles at L3 that the arm finishes its rotation (negative = early). */
    public static double l3ArmArrivalOffset() {
        return Preferences.getDouble(L3_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L3_ARM_ARRIVAL_OFFSET_SECONDS);
    }

    /** Seconds after the elevator settles at L4 that the arm finishes its rotation (negative = early). */
    public static double l4ArmArrivalOffset() {
        return Preferences.getDouble(L4_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L4_ARM_ARRIVAL_OFFSET_SECONDS);
    }
}
