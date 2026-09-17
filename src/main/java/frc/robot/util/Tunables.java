package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

import frc.robot.Constants.DriveConstants;
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
 * offsets, tracking gains, handoff heights). What does NOT belong here: the
 * measured mechanism contact geometry (tuck / low-box limits) - those are
 * physical facts, not tuning knobs - and motor-controller gains, which are
 * applied to the controllers at boot and are tuned live in the REV Hardware
 * Client / Phoenix Tuner X instead.
 *
 * Readers call the getters every time they need a value (a Preferences read
 * is one NetworkTables entry lookup - cheap), so edits take effect on the
 * next loop, or on the next button press for values the Superstructure
 * reads at plan time.
 */
public final class Tunables {
    private Tunables() {}

    // ------------------------------------------------------------------
    // Keys as shown in the Robot Preferences widget (grouped by prefix)
    // ------------------------------------------------------------------
    private static final String TELEOP_SPEED_SCALE = "Drive - Teleop Speed Scale (0-1)";
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
     * robot startup.
     */
    public static void init() {
        Preferences.initDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
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
        Preferences.setDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.setDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.setDouble(L1_SCORE_DISTANCE, VisionConstants.L1_SCORE_DISTANCE);
        Preferences.setDouble(REEF_BRANCH_OFFSET, VisionConstants.REEF_BRANCH_OFFSET);
        Preferences.setDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.setDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
        Preferences.setDouble(L3_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L3_ARM_ARRIVAL_OFFSET_SECONDS);
        Preferences.setDouble(L4_ARM_ARRIVAL_OFFSET, SuperstructureConstants.L4_ARM_ARRIVAL_OFFSET_SECONDS);
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
        double scale = Preferences.getDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        return Math.max(0.05, Math.min(1.0, scale));
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
