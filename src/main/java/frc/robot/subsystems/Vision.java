package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Superstructure;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraPose;
import frc.robot.Constants.VisionConstants.TagClass;
import frc.robot.util.Tunables;

/**
 * Vision subsystem: multi-Limelight AprilTag targeting and pose estimation.
 *
 * Pose estimation: while ENABLED the robot's heading is sent to each
 * Limelight every loop and the returned MegaTag2 poses are fused into the
 * swerve pose estimator with distance/tag-count based confidence. While
 * DISABLED (and for a short window after {@link #requestHeadingReseed()}
 * is called) MegaTag1 is fused instead, because its solve carries an absolute
 * heading from tag geometry - that is what seeds the heading MegaTag2 then
 * depends on. Every camera frame is fused at most once (the NT sample
 * timestamp identifies a frame), estimates are inserted oldest first so a
 * late camera cannot erase an earlier camera's correction, single-tag
 * MegaTag1 solves are gated on ambiguity and distance before their heading
 * is trusted, and a camera whose mounting pose is not measured never seeds
 * the heading.
 *
 * Targeting: each camera's primary tag is converted from Limelight camera
 * space into the ROBOT frame using that camera's mounting pose (lens
 * offset, yaw and pitch), so the tracking command servos on where the tag
 * is relative to the robot's center, whichever camera saw it. The tag's
 * field heading (from the AprilTag field layout) gives the heading at which
 * the robot is square to the tag's face. A camera only supplies the tag
 * classes it is mounted for (rear funnel camera: coral stations; reef
 * camera: reef; front barge camera: barge and processor), and only once its
 * lens pose is measured - the barge camera's pose is still a placeholder, so
 * it supplies nothing yet. Once tracking starts the first chosen tag is
 * latched so the goal cannot flip between adjacent reef faces mid-approach.
 *
 * The tracker never drives on a raw camera solve. A tag does not move, so
 * each NEW camera frame is turned into a FIELD position for the latched tag
 * - using the robot's pose at the moment the image was captured, not now -
 * and low-pass filtered there; every loop the tracker then gets that field
 * position seen from the robot's CURRENT odometry pose. Filtering a static
 * point adds no lag to the control, the 50-100 ms camera latency drops out
 * (driven on directly, it appears as a phantom lateral error whenever the
 * robot is turning: range x the heading change during the latency),
 * single-frame solve noise does not reach the wheels, and a dropped frame
 * changes nothing - which is also how the approach finishes when the latched tag
 * leaves the camera's view (the rear camera loses the station tag before
 * the bumpers are flush).
 *
 * The heading the tracker squares up to is handled the same way, and for
 * the same reason it must not depend on the pose estimator's heading being
 * FIELD-TRUE. The tag's square heading comes from the field layout, but the
 * pose heading is whatever frame the gyro was last zeroed in (a driver
 * re-zero, a boot orientation, a half-converged seed) - compare the two
 * directly and the robot squares up to the wrong direction, i.e. turns AWAY
 * from the tag. So each frame's MegaTag1 solve (which carries a field-true
 * heading from the tag geometry alone) is compared with the pose heading at
 * capture, the difference is filtered, and the square heading is handed to
 * the tracker already converted into the pose estimator's own frame.
 * Nothing here ever WRITES the pose heading.
 *
 * Latch rules: the latch is released when tracking is switched off, when
 * the latched tag has been out of view for TARGET_MEMORY_SECONDS, or at
 * once when the superstructure goal leaves the tag's family (a finished
 * score going home); after that last case nothing new is latched until
 * tracking is switched off and on again. L3 / L4 reef goals are approached
 * in two stages - a standoff REEF_STANDOFF_EXTRA back from flush until the
 * scoring pose is ready, then flush - see {@link #getTrackingGoal}.
 *
 * Separately from alignment, {@link #getClosestSeenTag()} and
 * {@link #getSeenTagsSummary()} report every tag every camera sees, with
 * nothing filtered, so the dashboard agrees with the camera streams.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class Vision extends SubsystemBase {

    /** Which reef branch to line up on for L2-L4 coral scoring. */
    public enum BranchSide {
        LEFT,
        RIGHT
    }

    /**
     * A resolved alignment goal: where the tag should sit in the ROBOT frame
     * (WPILib: +X forward, +Y left, meters) when the robot is in position.
     */
    public static class TrackingGoal {
        /** Desired robot-frame X of the tag; negative for a rear (backed-up) approach. */
        public final double forward;
        /** Desired robot-frame Y of the tag (+ = tag to the robot's LEFT). */
        public final double left;

        public TrackingGoal(double forward, double left) {
            this.forward = forward;
            this.left = left;
        }
    }

    /**
     * An AprilTag target: seen by a Limelight this loop, or (fromMemory) the
     * latched tag's last sighting carried on odometry. The camera-space
     * numbers are kept for display; the driving math uses
     * {@link #robotFrame} and {@link #squareHeading}.
     */
    public static class AprilTagTarget {
        public final int id;
        public final TagClass tagClass;
        public final String limelightName;
        public final int cameraIndex;
        public final double tx; // Horizontal angle to target (degrees, + = right), display only
        public final double ty; // Vertical angle to target (degrees), display only
        /** Raw Limelight camera-space position: X right, Y down, Z forward (meters). */
        public final double cameraX, cameraY, cameraZ;
        /** Tag position in the robot frame (+X forward, +Y left), meters. */
        public final Translation2d robotFrame;
        /** Field heading at which the robot is square to this tag's face (empty if the tag is not in the layout). */
        public final Optional<Rotation2d> squareHeading;
        /** True when no camera sees the tag and this is its last sighting carried on odometry. */
        public final boolean fromMemory;

        public AprilTagTarget(int id, TagClass tagClass, String limelightName, int cameraIndex,
                double tx, double ty, double cameraX, double cameraY, double cameraZ,
                Translation2d robotFrame, Optional<Rotation2d> squareHeading, boolean fromMemory) {
            this.id = id;
            this.tagClass = tagClass;
            this.limelightName = limelightName;
            this.cameraIndex = cameraIndex;
            this.tx = tx;
            this.ty = ty;
            this.cameraX = cameraX;
            this.cameraY = cameraY;
            this.cameraZ = cameraZ;
            this.robotFrame = robotFrame;
            this.squareHeading = squareHeading;
            this.fromMemory = fromMemory;
        }

        /** Ground-plane distance from the robot center to the tag, in meters. */
        public double distance() {
            return robotFrame.getNorm();
        }
    }

    private final Swerve swerve;

    private boolean trackingEnabled = false;
    private boolean positionTrackingEnabled = true;

    /** Driver-selected reef branch for L2-L4 alignment. */
    private BranchSide branchSide = BranchSide.LEFT;

    /**
     * Inputs to the L3 / L4 reef standoff (wired in RobotContainer): the
     * alignment holds OFF the reef while the scoring pose is not ready, or
     * while a finished score is waiting for room. See getTrackingGoal.
     */
    private BooleanSupplier poseReady = () -> true;
    private BooleanSupplier waitingForBackOff = () -> false;

    /** Where the superstructure goal comes from (wired in RobotContainer). */
    private Supplier<Superstructure.Goal> goalSupplier = () -> Superstructure.Goal.STOW;

    // Target caches, refreshed once per periodic()
    private Optional<AprilTagTarget> cachedBestTarget = Optional.empty();   // matches the current goal (drives the tracker)
    private Optional<AprilTagTarget> cachedBestVisible = Optional.empty();  // closest tag a camera may ALIGN on (robot frame)
    private Optional<SeenTag> cachedClosestSeen = Optional.empty();         // closest tag ANY camera reports (dashboard)
    private String cachedSeenTagsSummary = "";

    // Tag latch while tracking: -1 = none. The latched tag's last sighting is
    // remembered as a field position so odometry can carry it while unseen.
    private int latchedTagId = -1;
    private double latchedLastSeenTime = 0.0;
    private TagClass latchedTagClass = TagClass.NONE;
    private Optional<Rotation2d> latchedSquareHeading = Optional.empty();
    /** Filtered FIELD position of the latched tag (see the class note); null until its first sample. */
    private Translation2d latchedTagFieldPosition = null;
    /** NT timestamp (microseconds) of the last camera sample folded into the filter: one update per frame. */
    private long latchedSampleStamp = Long.MIN_VALUE;
    /** Consecutive samples that disagreed with the filter by more than the outlier distance. */
    private int latchedOutlierCount = 0;
    /** Filtered (field-true heading - pose estimator heading) while latched; null until the first MegaTag1 sample. */
    private Rotation2d latchedHeadingOffset = null;
    private int latchedHeadingOutlierCount = 0;
    /** True once this tracking session's latch was dropped because the goal left its tag family (cleared when tracking is switched off). */
    private boolean latchSpent = false;
    /** L3 / L4 two-stage approach: true once the scoring pose has been seen ready, until the score backs off (see getTrackingGoal). */
    private boolean closeInLatched = false;
    private Superstructure.Goal closeInGoal = null;

    // Precomputed "limelight-<name>" NT table names (avoids per-loop string
    // concatenation in the hot paths)
    private final String[] limelightTableNames;

    // Per-camera fusion bookkeeping (same indexing as LIMELIGHT_NAMES)
    private final double[] lastSeenTimestamp;      // NT timestamp of the last frame examined (dedupe)
    private final Pose2d[] lastFusedPose;          // last estimate fused from each camera (display)
    private final double[] lastFusedTime;          // FPGA time it was fused

    // Heading seed bookkeeping
    private double lastHeadingSeedTime = -1e9;        // FPGA time any trusted MegaTag1 solve was fused
    private double lastStrongHeadingSeedTime = -1e9;  // FPGA time a two-or-more-tag MegaTag1 solve was fused
    private Rotation2d lastStrongSeedHeading = null;  // that solve's own heading
    private double reseedUntil = -1.0;                // FPGA time until which MegaTag1 is fused while enabled

    // 2025 field layout (blue-alliance origin, matching the wpiBlue poses)
    private final AprilTagFieldLayout fieldLayout;

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        int n = VisionConstants.LIMELIGHT_NAMES.length;
        limelightTableNames = new String[n];
        lastSeenTimestamp = new double[n];
        lastFusedPose = new Pose2d[n];
        lastFusedTime = new double[n];
        for (int i = 0; i < n; i++) {
            limelightTableNames[i] = "limelight-" + VisionConstants.LIMELIGHT_NAMES[i];
            lastSeenTimestamp[i] = Double.NaN;
            lastFusedTime[i] = -1e9;
        }

        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load the AprilTag field layout; alignment falls back to tag bearing",
                ex.getStackTrace());
        }
        fieldLayout = layout;

        for (int i = 0; i < limelightTableNames.length; i++) {
            String tableName = limelightTableNames[i];
            // Set all Limelights to the AprilTag pipeline
            LimelightHelpers.setPipelineIndex(tableName, VisionConstants.APRILTAG_PIPELINE);
            // LEDs stay OFF permanently: AprilTags are detected in ambient
            // light and gain nothing from illumination, while the LED array
            // is the camera's single largest heat source (fan noise).
            LimelightHelpers.setLEDMode_ForceOff(tableName);
            // Push the measured lens position/orientation so MegaTag's
            // robot pose is computed from the right camera offset (unmeasured
            // cameras keep their web-UI configuration).
            CameraPose pose = VisionConstants.LIMELIGHT_POSES[i];
            if (pose.measured) {
                LimelightHelpers.setCameraPose_RobotSpace(tableName,
                    pose.forwardMeters, pose.sideMeters, pose.upMeters,
                    pose.rollDegrees, pose.pitchDegrees, pose.yawDegrees);
            }
            LimelightHelpers.setPriorityTagID(tableName, -1);
        }
    }

    // ------------------------------------------------------------------
    // Geometry (static, unit-tested)
    // ------------------------------------------------------------------

    /**
     * Converts a Limelight camera-space tag position (X right, Y down,
     * Z forward along the optical axis, meters) into the robot frame
     * (+X forward, +Y left) using the camera's mounting pose. The pitch
     * tilts the optical axis, so the ground-plane distance along the
     * camera's heading is Z*cos(pitch) + Y*sin(pitch); the yaw and the lens
     * offset then place the point on the robot.
     */
    public static Translation2d tagPositionInRobotFrame(CameraPose camera, double cameraX, double cameraY, double cameraZ) {
        double pitch = Math.toRadians(camera.pitchDegrees);
        double forwardAlongCameraHeading = cameraZ * Math.cos(pitch) + cameraY * Math.sin(pitch);
        double leftOfCameraHeading = -cameraX;
        Translation2d inCameraHeadingFrame = new Translation2d(forwardAlongCameraHeading, leftOfCameraHeading)
            .rotateBy(Rotation2d.fromDegrees(camera.yawDegrees));
        return camera.lensOnRobot().plus(inCameraHeadingFrame);
    }

    /** A robot-frame point (+X forward, +Y left) expressed as a field position, given the robot's field pose. */
    public static Translation2d robotFrameToField(Pose2d robot, Translation2d inRobotFrame) {
        return robot.getTranslation().plus(inRobotFrame.rotateBy(robot.getRotation()));
    }

    /** A field position expressed in the robot frame (+X forward, +Y left), given the robot's field pose. */
    public static Translation2d fieldToRobotFrame(Pose2d robot, Translation2d onField) {
        return onField.minus(robot.getTranslation()).rotateBy(robot.getRotation().unaryMinus());
    }

    /**
     * Field heading at which the robot is square to the given tag's face:
     * facing it for reef tags, backed up to it for coral-station tags.
     */
    public static Optional<Rotation2d> squareHeading(AprilTagFieldLayout layout, int tagId, TagClass tagClass) {
        if (layout == null) {
            return Optional.empty();
        }
        return layout.getTagPose(tagId).map(pose -> {
            Rotation2d tagFacing = pose.toPose2d().getRotation();
            return tagClass == TagClass.CORAL_STATION ? tagFacing : tagFacing.plus(Rotation2d.k180deg);
        });
    }

    /** The class of a 2025 Reefscape tag: reef, coral station, or NONE (barge / processor / unknown). */
    public static TagClass classOf(int tagId) {
        for (int tag : VisionConstants.REEF_TAGS) {
            if (tag == tagId) {
                return TagClass.REEF;
            }
        }
        for (int tag : VisionConstants.CORAL_STATION_TAGS) {
            if (tag == tagId) {
                return TagClass.CORAL_STATION;
            }
        }
        for (int tag : VisionConstants.BARGE_TAGS) {
            if (tag == tagId) {
                return TagClass.BARGE;
            }
        }
        for (int tag : VisionConstants.PROCESSOR_TAGS) {
            if (tag == tagId) {
                return TagClass.PROCESSOR;
            }
        }
        return TagClass.NONE;
    }

    // ------------------------------------------------------------------
    // Throttle
    // ------------------------------------------------------------------

    /**
     * Frame throttle currently applied to the cameras (-1 = not yet sent).
     * Tracked so the NT write happens only when the throttle changes.
     */
    private int appliedThrottle = -1;

    /**
     * Throttles AprilTag processing while the robot is disabled and restores
     * full rate when enabled - the cameras spend most of their powered-on
     * life disabled in the pit, where full-rate processing only makes heat.
     * Selecting TEST mode on the Driver Station lifts the throttle without
     * enabling: throttled, every vision readout on the dashboard is a second
     * or more behind the camera's own stream, too slow for checking a camera
     * on the bench.
     */
    private void updateThrottle() {
        int throttle = DriverStation.isDisabled() && !DriverStation.isTest()
            ? VisionConstants.DISABLED_THROTTLE
            : VisionConstants.ENABLED_THROTTLE;
        if (throttle != appliedThrottle) {
            for (String tableName : limelightTableNames) {
                LimelightHelpers.SetThrottle(tableName, throttle);
            }
            appliedThrottle = throttle;
        }
    }

    // ------------------------------------------------------------------
    // Tracking state
    // ------------------------------------------------------------------

    /** Whether the Limelight at the given LIMELIGHT_NAMES index sees a target. */
    public boolean hasTarget(int index) {
        return LimelightHelpers.getTV(limelightTableNames[index]);
    }

    /**
     * Records whether AprilTag tracking is active. Deliberately does NOT
     * touch the LEDs: they add nothing to AprilTag detection and are the
     * camera's largest heat source (and the cause of its fan noise).
     * Turning tracking off releases the tag latch.
     */
    public void toggleTracking(boolean enabled) {
        trackingEnabled = enabled;
        if (!enabled) {
            releaseLatch();
        }
    }

    /**
     * Whether AprilTag tracking is enabled. Intentionally unused: kept as
     * the getter paired with toggleTracking() for dashboards and tests
     * (Swerve keeps its own copy of this state for the align triggers).
     */
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    /** Wires in "hold the L3 / L4 alignment off the reef" (see getTrackingGoal). */
    public void setReefStandoffSuppliers(BooleanSupplier poseReady, BooleanSupplier waitingForBackOff) {
        this.poseReady = poseReady;
        this.waitingForBackOff = waitingForBackOff;
    }

    /** Wires in the superstructure goal used to resolve tracking goals. */
    public void setGoalSupplier(Supplier<Superstructure.Goal> supplier) {
        this.goalSupplier = supplier;
    }

    /** Selects which reef branch L2-L4 alignment centers on. */
    public void setBranchSide(BranchSide side) {
        this.branchSide = side;
    }

    public BranchSide getBranchSide() {
        return branchSide;
    }

    /** The tag id the tracker is latched onto, or -1. */
    public int getLatchedTagId() {
        return latchedTagId;
    }

    /**
     * Resolves the alignment goal for a target, combining the tag's class
     * with the superstructure's current goal and the selected branch side.
     * Goals are in the ROBOT frame: forward = where the tag should be along
     * +X (negative = behind the robot, i.e. a backed-up approach), left =
     * where it should be along +Y.
     *
     * Reef goals:
     *  - CORAL_L1: centered, held L1_SCORE_DISTANCE away so the arm can
     *    swing to its 100-degree pose without hitting the reef.
     *  - CORAL_L2/L3/L4: bumpers flush, robot centered on the selected
     *    branch. The LEFT branch is REEF_BRANCH_OFFSET to the robot's left
     *    of the tag, so centering on it puts the tag that far to the
     *    robot's RIGHT (negative left). L3 / L4 hold REEF_STANDOFF_EXTRA
     *    back from flush until the scoring pose is ready (see below).
     *  - Everything else (algae intakes, stow, ...): bumpers flush, centered.
     *
     * Coral station goals: rear bumpers flush with the wall, centered
     * (approached backward, so the tag sits BEHIND the robot center).
     *
     * Barge / processor goals: centered on the tag, held
     * BARGE_SCORE_DISTANCE / PROCESSOR_DISTANCE away (both first guesses,
     * and unused until the barge camera's mounting pose is measured).
     *
     * @return the goal, or empty for a tag class that has none
     */
    public Optional<TrackingGoal> getTrackingGoal(AprilTagTarget target) {
        switch (target.tagClass) {
            case REEF:
                switch (goalSupplier.get()) {
                    case CORAL_L1:
                        return Optional.of(new TrackingGoal(Tunables.l1ScoreDistance(), 0.0));
                    case CORAL_L2:
                    case CORAL_L3:
                    case CORAL_L4:
                        double branchOffset = Tunables.reefBranchOffset();
                        double left = branchSide == BranchSide.LEFT ? -branchOffset : branchOffset;
                        // L3 / L4 in two stages. Getting the arm into or out
                        // of those poses swings the claw 9-12 in past the
                        // front bumper, so the robot lines up on the branch a
                        // standoff back, waits there for the pose, and only
                        // then closes to flush; after the score it backs out
                        // to the standoff again, which is also what releases
                        // the automatic stow. L2 never pokes out: flush at once.
                        //
                        // The decision to close in is LATCHED per approach.
                        // "Pose ready" read raw goes false for a loop whenever
                        // the arm or the carriage twitches - the bumper
                        // touching the reef, the rollers ejecting - and each
                        // time the goal would jump REEF_STANDOFF_EXTRA back
                        // out: the robot backs off, comes back in, bumps, and
                        // repeats. The latch resets on a new level or when a
                        // finished score is waiting for the back-off.
                        Superstructure.Goal level = goalSupplier.get();
                        if (level != closeInGoal || waitingForBackOff.getAsBoolean()) {
                            closeInLatched = false;
                            closeInGoal = level;
                        }
                        if (!waitingForBackOff.getAsBoolean() && poseReady.getAsBoolean()) {
                            closeInLatched = true;
                        }
                        boolean holdOff = level != Superstructure.Goal.CORAL_L2 && !closeInLatched;
                        double forward = Tunables.reefFlushDistance()
                            + (holdOff ? VisionConstants.REEF_STANDOFF_EXTRA : 0.0);
                        return Optional.of(new TrackingGoal(forward, left));
                    default:
                        return Optional.of(new TrackingGoal(Tunables.reefFlushDistance(), 0.0));
                }
            case CORAL_STATION:
                return Optional.of(new TrackingGoal(-Tunables.stationFlushDistance(), 0.0));
            // Barge and processor: centered on the tag and square to it,
            // whichever branch trigger the driver is holding.
            case BARGE:
                return Optional.of(new TrackingGoal(Tunables.bargeScoreDistance(), 0.0));
            case PROCESSOR:
                return Optional.of(new TrackingGoal(Tunables.processorDistance(), 0.0));
            default:
                return Optional.empty();
        }
    }

    /** The target the tracker should drive to this loop (cached; refreshed in periodic). */
    public Optional<AprilTagTarget> getBestTarget() {
        return cachedBestTarget;
    }

    /**
     * The closest tag a camera may ALIGN on this loop (its class is one the
     * camera is mounted for, and the camera's lens pose is measured),
     * regardless of the superstructure goal or the latch. It carries a
     * robot-frame position, so it feeds the distance / lateral readouts used
     * to measure the flush distances with the robot pushed into position,
     * and the near-reef guard. For "which tag do the cameras see" use
     * {@link #getClosestSeenTag()}, which filters nothing.
     */
    public Optional<AprilTagTarget> getBestVisibleTarget() {
        return cachedBestVisible;
    }

    // ------------------------------------------------------------------
    // What the cameras see (dashboard)
    // ------------------------------------------------------------------

    /**
     * One tag exactly as a camera reports it, with nothing filtered: no tag
     * class, no camera role, no mounting pose, no latch. This is what the
     * camera's own stream draws, which is what a "which tag do you see"
     * readout has to agree with. The alignment caches above are a strict
     * subset of it - a camera only ALIGNS on the classes it is mounted for,
     * and only once its lens pose is measured.
     */
    public static final class SeenTag {
        public final int id;
        public final int cameraIndex;
        /** Camera to tag, meters; 0 when the camera has no 3D solve for it. */
        public final double distanceMeters;
        /** Share of the image the tag covers, percent. */
        public final double area;

        SeenTag(int id, int cameraIndex, double distanceMeters, double area) {
            this.id = id;
            this.cameraIndex = cameraIndex;
            this.distanceMeters = distanceMeters;
            this.area = area;
        }
    }

    /** Values per tag in a Limelight "rawfiducials" array: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity. */
    private static final int RAW_FIDUCIAL_STRIDE = 7;

    /**
     * The closest tag in the cameras' "rawfiducials" arrays (one per camera,
     * null or empty = that camera reports nothing). Closest by the 3D
     * distance when any tag has one; by image area (largest) when none does,
     * so the readout still works on a pipeline with the 3D solve off.
     */
    public static Optional<SeenTag> closestSeenTag(double[][] rawFiducialsByCamera) {
        SeenTag byDistance = null;
        SeenTag byArea = null;
        for (int camera = 0; camera < rawFiducialsByCamera.length; camera++) {
            double[] raw = rawFiducialsByCamera[camera];
            if (raw == null || raw.length % RAW_FIDUCIAL_STRIDE != 0) {
                continue;
            }
            for (int base = 0; base < raw.length; base += RAW_FIDUCIAL_STRIDE) {
                SeenTag tag = new SeenTag((int) raw[base], camera, raw[base + 4], raw[base + 3]);
                if (tag.distanceMeters > 0 && (byDistance == null || tag.distanceMeters < byDistance.distanceMeters)) {
                    byDistance = tag;
                }
                if (byArea == null || tag.area > byArea.area) {
                    byArea = tag;
                }
            }
        }
        return Optional.ofNullable(byDistance != null ? byDistance : byArea);
    }

    /** Every tag ID per camera, in the cameras' own order: "funnel: 12 | reef: 18 19" ("" = none). */
    public static String seenTagsSummary(String[] cameraNames, double[][] rawFiducialsByCamera) {
        StringBuilder summary = new StringBuilder();
        for (int camera = 0; camera < rawFiducialsByCamera.length; camera++) {
            double[] raw = rawFiducialsByCamera[camera];
            if (raw == null || raw.length == 0 || raw.length % RAW_FIDUCIAL_STRIDE != 0) {
                continue;
            }
            if (summary.length() > 0) {
                summary.append(" | ");
            }
            summary.append(cameraNames[camera]).append(':');
            for (int base = 0; base < raw.length; base += RAW_FIDUCIAL_STRIDE) {
                summary.append(' ').append((int) raw[base]);
            }
        }
        return summary.toString();
    }

    private final double[][] rawFiducialsByCamera = new double[VisionConstants.LIMELIGHT_NAMES.length][];

    /**
     * Reads what every camera reports this loop. A camera whose array has
     * not changed for SEEN_TAG_STALE_SECONDS is treated as reporting nothing:
     * NetworkTables keeps the last value of a camera that lost power or its
     * link, and a tag in view never yields two identical arrays in a row.
     */
    private void refreshSeenTags() {
        double now = Timer.getFPGATimestamp();
        for (int i = 0; i < limelightTableNames.length; i++) {
            var sample = LimelightHelpers.getLimelightDoubleArrayEntry(limelightTableNames[i], "rawfiducials").getAtomic();
            boolean stale = now - sample.timestamp / 1e6 > VisionConstants.SEEN_TAG_STALE_SECONDS;
            rawFiducialsByCamera[i] = stale ? null : sample.value;
        }
        cachedClosestSeen = closestSeenTag(rawFiducialsByCamera);
        cachedSeenTagsSummary = seenTagsSummary(VisionConstants.LIMELIGHT_NAMES, rawFiducialsByCamera);
    }

    /** The closest tag any camera reports this loop, unfiltered - what the dashboard's "Best Tag" shows. */
    public Optional<SeenTag> getClosestSeenTag() {
        return cachedClosestSeen;
    }

    /** Every tag ID each camera reports this loop ("funnel: 12 | reef: 18 19"), "" when none. */
    public String getSeenTagsSummary() {
        return cachedSeenTagsSummary;
    }

    /**
     * True when the tag's class matches what the superstructure is doing:
     * a STOW goal means the robot is heading to a coral station to intake,
     * so only station tags are candidates; carrying an algae or holding the
     * barge pose targets the barge or the processor; every other goal
     * (coral levels, algae intakes) targets the reef. Without this, a station
     * tag seen by the rear camera at 3 m could out-"close" the intended
     * reef tag at 4 m and hijack a scoring alignment.
     */
    private boolean tagMatchesCurrentGoal(TagClass tagClass) {
        switch (goalSupplier.get()) {
            case STOW:
                return tagClass == TagClass.CORAL_STATION;
            // Carrying an algae, or at the barge pose: the next stop is the
            // barge or the processor - whichever of the two is in view.
            case ALGAE_CARRY:
            case ALGAE_SCORE:
                return tagClass == TagClass.BARGE || tagClass == TagClass.PROCESSOR;
            default:
                return tagClass == TagClass.REEF;
        }
    }

    private static boolean cameraSupplies(int cameraIndex, TagClass tagClass) {
        for (TagClass allowed : VisionConstants.LIMELIGHT_TRACKING_CLASSES[cameraIndex]) {
            if (allowed == tagClass) {
                return true;
            }
        }
        return false;
    }

    private void latch(AprilTagTarget target, long sampleStamp, double captureTime, double fieldTrueYawDegrees,
            Pose2d robotPose) {
        latchedTagId = target.id;
        latchedTagClass = target.tagClass;
        latchedSquareHeading = target.squareHeading;
        latchedTagFieldPosition = null;
        latchedSampleStamp = Long.MIN_VALUE;
        latchedOutlierCount = 0;
        latchedHeadingOffset = null;
        latchedHeadingOutlierCount = 0;
        updateLatchedEstimate(target, sampleStamp, captureTime, fieldTrueYawDegrees, robotPose);
        for (String tableName : limelightTableNames) {
            LimelightHelpers.setPriorityTagID(tableName, target.id);
        }
    }

    /**
     * Folds one sighting of the latched tag into its filtered field
     * position. Only a NEW camera frame counts (the robot loop reads the
     * same frame several times), and the frame is placed on the field with
     * the pose the robot had when the image was CAPTURED. A sample far from
     * the estimate is ignored unless it persists - one bad solve must not
     * yank the goal, but a real change (a pose reset) is followed.
     */
    private void updateLatchedEstimate(AprilTagTarget target, long sampleStamp, double captureTime,
            double fieldTrueYawDegrees, Pose2d robotPose) {
        latchedLastSeenTime = Timer.getFPGATimestamp();
        if (sampleStamp == latchedSampleStamp) {
            return;
        }
        latchedSampleStamp = sampleStamp;
        Pose2d poseAtCapture = swerve.samplePoseAt(Utils.fpgaToCurrentTime(captureTime)).orElse(robotPose);
        updateHeadingOffset(fieldTrueYawDegrees, poseAtCapture.getRotation());
        Translation2d measured = robotFrameToField(poseAtCapture, target.robotFrame);
        if (latchedTagFieldPosition == null) {
            latchedTagFieldPosition = measured;
            return;
        }
        if (measured.getDistance(latchedTagFieldPosition) > VisionConstants.TrackingGains.TARGET_OUTLIER_METERS
                && ++latchedOutlierCount < VisionConstants.TrackingGains.TARGET_OUTLIER_FRAMES) {
            return;
        }
        latchedTagFieldPosition = latchedOutlierCount > 0
            ? measured
            : latchedTagFieldPosition.interpolate(measured, VisionConstants.TrackingGains.TARGET_FILTER_ALPHA);
        latchedOutlierCount = 0;
    }

    /**
     * Folds one frame's field-true heading (MegaTag1, NaN when that frame
     * had no solve) into the filtered offset between the field-true heading
     * and the pose estimator's. A single-tag solve can flip (pose
     * ambiguity), so a sample far from the estimate only counts once it
     * persists.
     */
    private void updateHeadingOffset(double fieldTrueYawDegrees, Rotation2d poseHeadingAtCapture) {
        if (Double.isNaN(fieldTrueYawDegrees)) {
            return;
        }
        Rotation2d measured = Rotation2d.fromDegrees(fieldTrueYawDegrees).minus(poseHeadingAtCapture);
        if (latchedHeadingOffset == null) {
            latchedHeadingOffset = measured;
            return;
        }
        Rotation2d innovation = measured.minus(latchedHeadingOffset);
        if (Math.abs(innovation.getDegrees()) > VisionConstants.TrackingGains.HEADING_OUTLIER_DEGREES
                && ++latchedHeadingOutlierCount < VisionConstants.TrackingGains.HEADING_OUTLIER_FRAMES) {
            return;
        }
        latchedHeadingOffset = latchedHeadingOutlierCount > 0
            ? measured
            : latchedHeadingOffset.plus(innovation.times(VisionConstants.TrackingGains.HEADING_FILTER_ALPHA));
        latchedHeadingOutlierCount = 0;
    }

    /**
     * The latched tag's square heading in the POSE ESTIMATOR's frame (what
     * the tracker compares with the pose heading). Falls back to the layout
     * heading as it is until a MegaTag1 sample has arrived.
     */
    private Optional<Rotation2d> squareHeadingInPoseFrame() {
        if (latchedHeadingOffset == null) {
            return latchedSquareHeading;
        }
        return latchedSquareHeading.map(square -> square.minus(latchedHeadingOffset));
    }

    /** Field-true minus pose-estimator heading while latched, degrees (0 when unknown) - for the dashboard. */
    public double getLatchedHeadingOffsetDegrees() {
        return latchedHeadingOffset == null ? 0.0 : latchedHeadingOffset.getDegrees();
    }

    private void releaseLatch() {
        closeInLatched = false;
        if (latchedTagId < 0) {
            return;
        }
        latchedTagId = -1;
        latchedTagFieldPosition = null;
        latchedHeadingOffset = null;
        for (String tableName : limelightTableNames) {
            LimelightHelpers.setPriorityTagID(tableName, -1);
        }
    }

    /**
     * The latched tag as the tracker should see it: its filtered field
     * position from the robot's current pose. {@code live} is this loop's
     * camera sighting (its display numbers are kept), or null while the tag
     * is out of view.
     */
    private AprilTagTarget carriedTarget(Pose2d robotPose, AprilTagTarget live) {
        Translation2d inRobot = fieldToRobotFrame(robotPose, latchedTagFieldPosition);
        if (live == null) {
            return new AprilTagTarget(latchedTagId, latchedTagClass, "memory", -1,
                0.0, 0.0, 0.0, 0.0, 0.0, inRobot, squareHeadingInPoseFrame(), true);
        }
        return new AprilTagTarget(live.id, live.tagClass, live.limelightName, live.cameraIndex,
            live.tx, live.ty, live.cameraX, live.cameraY, live.cameraZ, inRobot, squareHeadingInPoseFrame(), false);
    }

    /**
     * Queries every Limelight and refreshes both alignment caches: the
     * closest tag a camera may align on, whatever the goal (the dashboard's
     * "Alignment Tag" readouts and the near-reef guard), and the closest tag
     * matching the current goal (tracker), honoring the latch while tracking. While
     * the latched tag is unseen (for up to TARGET_MEMORY_SECONDS) its last
     * sighting carried on odometry stands in, so an approach whose tag
     * leaves the camera's view at the end still finishes.
     */
    private void refreshTargets(Pose2d robotPose) {
        double now = Timer.getFPGATimestamp();
        AprilTagTarget bestVisible = null;
        double bestVisibleDistance = Double.MAX_VALUE;
        AprilTagTarget bestForGoal = null;
        double bestGoalDistance = Double.MAX_VALUE;
        long bestGoalStamp = 0;
        double bestGoalCaptureTime = now;
        double bestGoalFieldTrueYaw = Double.NaN;
        boolean latchedSeen = false;

        for (int i = 0; i < limelightTableNames.length; i++) {
            if (!VisionConstants.LIMELIGHT_POSES[i].measured) {
                continue; // alignment needs the lens pose: an unmeasured camera supplies nothing
            }
            String limelightName = limelightTableNames[i];
            if (!LimelightHelpers.getTV(limelightName)) {
                continue;
            }
            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            TagClass tagClass = classOf(tagId);
            if (tagClass == TagClass.NONE || !cameraSupplies(i, tagClass)) {
                continue; // not a class this camera is mounted for
            }
            // Raw camera-space array: an empty array (no 3D solve / topic
            // absent) or an all-zero one (3D solve disabled) must not become
            // a "0 m away" target that wins the closest-tag selection.
            // Read atomically with its NT timestamp: the timestamp identifies
            // the camera frame, and minus the pipeline + capture latency it
            // is when the image was taken.
            var sample = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "targetpose_cameraspace").getAtomic();
            double[] cameraSpace = sample.value;
            if (cameraSpace.length < 6 || !(cameraSpace[2] > VisionConstants.MIN_CAMERA_Z)) {
                continue;
            }
            Translation2d inRobot = tagPositionInRobotFrame(VisionConstants.LIMELIGHT_POSES[i],
                cameraSpace[0], cameraSpace[1], cameraSpace[2]);
            AprilTagTarget target = new AprilTagTarget(
                tagId, tagClass, VisionConstants.LIMELIGHT_NAMES[i], i,
                LimelightHelpers.getTX(limelightName), LimelightHelpers.getTY(limelightName),
                cameraSpace[0], cameraSpace[1], cameraSpace[2],
                inRobot, squareHeading(fieldLayout, tagId, tagClass), false);
            double distance = target.distance();
            if (distance < bestVisibleDistance) {
                bestVisible = target;
                bestVisibleDistance = distance;
            }
            if (!tagMatchesCurrentGoal(tagClass)) {
                continue;
            }
            if (latchedTagId >= 0 && tagId != latchedTagId) {
                continue; // latched onto another tag
            }
            if (tagId == latchedTagId) {
                latchedSeen = true;
            }
            if (distance < bestGoalDistance) {
                bestForGoal = target;
                bestGoalDistance = distance;
                bestGoalStamp = sample.timestamp;
                bestGoalCaptureTime = sample.timestamp / 1e6
                    - (LimelightHelpers.getLatency_Pipeline(limelightName)
                        + LimelightHelpers.getLatency_Capture(limelightName)) / 1000.0;
                // MegaTag1 robot pose for the same frame: [x, y, z, roll,
                // pitch, YAW, latency, TAG COUNT, ...], blue origin, degrees.
                // Its yaw is field-true from the tag geometry alone. Only a
                // camera with a measured mounting pose can supply it.
                double[] mt1 = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "botpose_wpiblue").get();
                bestGoalFieldTrueYaw = VisionConstants.LIMELIGHT_POSES[i].measured && mt1.length >= 8 && mt1[7] >= 1
                    ? mt1[5] : Double.NaN;
            }
        }

        cachedBestVisible = Optional.ofNullable(bestVisible);
        cachedBestTarget = Optional.ofNullable(bestForGoal);

        // Latch management: hold the first chosen tag while tracking, fold
        // each new frame of it into the filtered field position, and drop it
        // once it has been out of view for the memory time. While latched
        // the tracker ALWAYS gets the filtered position seen from the
        // current odometry pose - seen this loop or not.
        if (!trackingEnabled) {
            releaseLatch();
            latchSpent = false;
            return;
        }
        // The goal has left the latched tag's family (the score finished and
        // the Superstructure went home: STOW is a station goal, the latched
        // tag is a reef tag). Drop it NOW rather than letting it live on in
        // memory for TARGET_MEMORY_SECONDS: a remembered reef tag with a
        // non-reef goal resolves to "flush and centered", so with the align
        // trigger still held the robot would drive back INTO the reef at the
        // moment the L3 / L4 exit is swinging the claw out past the bumper.
        // Nothing new is latched until the trigger is pressed again: holding
        // it through a score must not send the robot off to a coral station
        // by itself.
        if (latchedTagId >= 0 && !tagMatchesCurrentGoal(latchedTagClass)) {
            releaseLatch();
            latchSpent = true;
        }
        if (latchSpent) {
            cachedBestTarget = Optional.empty();
            return;
        }
        if (latchedTagId < 0) {
            if (bestForGoal != null) {
                latch(bestForGoal, bestGoalStamp, bestGoalCaptureTime, bestGoalFieldTrueYaw, robotPose);
            }
        } else if (latchedSeen) {
            updateLatchedEstimate(bestForGoal, bestGoalStamp, bestGoalCaptureTime, bestGoalFieldTrueYaw, robotPose);
        } else if (now - latchedLastSeenTime > VisionConstants.TrackingGains.TARGET_MEMORY_SECONDS) {
            releaseLatch();
        }
        if (latchedTagId >= 0 && latchedTagFieldPosition != null) {
            cachedBestTarget = Optional.of(carriedTarget(robotPose, latchedSeen ? bestForGoal : null));
        }
    }

    // ------------------------------------------------------------------
    // Pose fusion
    // ------------------------------------------------------------------

    /**
     * Enables or disables MegaTag pose fusion. Useful during testing to
     * compare pure wheel odometry against vision-corrected odometry.
     *
     * NOTE: intentionally unused (with its getter below) - fusion defaults
     * to on; call or temporarily bind this in test sessions only.
     */
    public void enablePositionTracking(boolean enabled) {
        positionTrackingEnabled = enabled;
    }

    public boolean isPositionTrackingEnabled() {
        return positionTrackingEnabled;
    }

    /**
     * Asks for the heading to be re-seeded from tag geometry: MegaTag1 (whose
     * solve carries an absolute heading) is fused for the next
     * HEADING_RESEED_WINDOW_SECONDS even though the robot is enabled. Meant
     * as a "fix my field-centric heading" action while a tag is in view;
     * unlike a gyro re-zero it cannot feed MegaTag2 a made-up heading.
     *
     * NOTE: currently not bound to a button (the driver's left bumper
     * zeroes the driver's heading frame - see Swerve.zeroDriverHeading).
     */
    public void requestHeadingReseed() {
        reseedUntil = Timer.getFPGATimestamp() + VisionConstants.HEADING_RESEED_WINDOW_SECONDS;
    }

    /** True while a driver-requested heading re-seed window is open. */
    public boolean isReseedingHeading() {
        return Timer.getFPGATimestamp() < reseedUntil;
    }

    /**
     * True when a MegaTag1 solve with a trusted heading (two or more tags,
     * or one close unambiguous tag) was fused within
     * HEADING_SEED_FRESHNESS_SECONDS - i.e. the pose heading has been
     * pulled toward a field-referenced value recently.
     */
    public boolean hasFreshHeadingSeed() {
        return Timer.getFPGATimestamp() - lastHeadingSeedTime <= VisionConstants.HEADING_SEED_FRESHNESS_SECONDS;
    }

    /**
     * True when the pose heading is CONVERGED on a strong seed: a
     * two-or-more-tag MegaTag1 solve was fused within
     * HEADING_SEED_FRESHNESS_SECONDS and the estimator's heading now agrees
     * with that solve's own heading within HEADING_SEED_AGREEMENT_DEGREES.
     * (One fused solve only closes part of the heading error, so freshness
     * alone would trust a heading that is still converging.)
     */
    public boolean hasStrongHeadingSeed() {
        if (lastStrongSeedHeading == null
                || Timer.getFPGATimestamp() - lastStrongHeadingSeedTime > VisionConstants.HEADING_SEED_FRESHNESS_SECONDS) {
            return false;
        }
        Rotation2d current = swerve.getStateCopy().Pose.getRotation();
        return Math.abs(current.minus(lastStrongSeedHeading).getDegrees()) <= VisionConstants.HEADING_SEED_AGREEMENT_DEGREES;
    }

    /** The last pose fused from the given camera, if it was fused within the last second. */
    public Optional<Pose2d> getLastFusedPose(int index) {
        if (lastFusedPose[index] == null || Timer.getFPGATimestamp() - lastFusedTime[index] > 1.0) {
            return Optional.empty();
        }
        return Optional.of(lastFusedPose[index]);
    }

    /**
     * Rejects estimates that cannot be right: NaN or off-field poses, far
     * MegaTag2 solves, and single-tag MegaTag1 solves that are ambiguous or
     * far (the classic single-tag pose flip would otherwise seed a heading
     * that is wrong by tens of degrees).
     */
    private static boolean isPlausible(PoseEstimate estimate, boolean megaTag1) {
        Translation2d t = estimate.pose.getTranslation();
        if (Double.isNaN(t.getX()) || Double.isNaN(t.getY())) {
            return false;
        }
        double margin = VisionConstants.FIELD_BOUNDS_MARGIN_METERS;
        if (t.getX() < -margin || t.getX() > VisionConstants.FIELD_LENGTH_METERS + margin
                || t.getY() < -margin || t.getY() > VisionConstants.FIELD_WIDTH_METERS + margin) {
            return false;
        }
        if (megaTag1) {
            if (estimate.tagCount < 2) {
                if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
                    return false;
                }
                RawFiducial fiducial = estimate.rawFiducials[0];
                if (fiducial.ambiguity > VisionConstants.MT1_SINGLE_TAG_MAX_AMBIGUITY
                        || fiducial.distToCamera > VisionConstants.MT1_SINGLE_TAG_MAX_DISTANCE_METERS) {
                    return false;
                }
            }
        } else if (estimate.avgTagDist > VisionConstants.MT2_MAX_AVG_TAG_DISTANCE_METERS) {
            return false;
        }
        return true;
    }

    /**
     * Fuses vision pose estimates from every Limelight into the drivetrain
     * odometry. Timestamp conversion to the Phoenix timebase happens inside
     * Swerve.addVisionMeasurement().
     *
     * Two modes:
     *  - DISABLED (pre-match / between periods) or a driver re-seed window:
     *    MegaTag1, whose solve includes an absolute HEADING from tag
     *    geometry alone. Its rotation is fused so the pose heading converges
     *    to field-correct while the robot sits still - without this, the
     *    heading MegaTag2 depends on would start at whatever the gyro booted
     *    to and every fused pose would be wrong until a manual reset. Only
     *    cameras with a measured mounting pose take part: a placeholder
     *    lens pose would seed a biased heading.
     *  - ENABLED: MegaTag2, which takes our (now-seeded) heading and returns
     *    a far more stable translation than single-tag solves. Its heading
     *    is our own gyro echoed back, so it gets effectively zero weight.
     *
     * Each camera frame is fused once: the estimator applies its correction
     * on every call, so re-adding the same sample every 20 ms loop (the
     * camera publishes at most ~90 Hz, and only ~1 Hz while throttled) would
     * collapse the estimate onto the raw camera pose regardless of the
     * standard deviations. The batch is inserted oldest first because
     * inserting an older measurement discards the corrections from newer
     * ones. Frames that arrive while the drivetrain is rejecting vision
     * (just after a fast spin) are dropped, not deferred.
     */
    private void updateRobotPosition(Pose2d robotPose) {
        double now = Timer.getFPGATimestamp();
        boolean seedingHeading = DriverStation.isDisabled() || now < reseedUntil;
        boolean rejecting = swerve.isRejectingVision();
        double headingDegrees = robotPose.getRotation().getDegrees();

        List<PoseEstimate> batch = new ArrayList<>(limelightTableNames.length);
        List<Integer> batchCameras = new ArrayList<>(limelightTableNames.length);
        for (int i = 0; i < limelightTableNames.length; i++) {
            String tableName = limelightTableNames[i];
            // NoFlush variant: one NT flush after the loop instead of a full
            // network flush per camera per loop
            LimelightHelpers.SetRobotOrientation_NoFlush(tableName, headingDegrees, 0, 0, 0, 0, 0);

            if (seedingHeading && !VisionConstants.LIMELIGHT_POSES[i].measured) {
                continue; // an unknown lens pose cannot seed the heading (or the translation)
            }

            PoseEstimate estimate = seedingHeading
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tableName);

            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }
            // Same NT sample as last loop: this frame has already been
            // considered (an unchanged sample reads back a bit-identical
            // timestamp; every new frame gets a new server timestamp).
            if (estimate.timestampSeconds == lastSeenTimestamp[i]) {
                continue;
            }
            lastSeenTimestamp[i] = estimate.timestampSeconds;
            if (rejecting || !isPlausible(estimate, seedingHeading)) {
                continue;
            }
            batch.add(estimate);
            batchCameras.add(i);
        }

        // Oldest first, so a late camera cannot erase an earlier camera's correction
        Integer[] order = new Integer[batch.size()];
        for (int k = 0; k < order.length; k++) {
            order[k] = k;
        }
        java.util.Arrays.sort(order, Comparator.comparingDouble(k -> batch.get(k).timestampSeconds));

        for (int k : order) {
            PoseEstimate estimate = batch.get(k);
            int camera = batchCameras.get(k);
            // Confidence scales with tag count and closeness
            double xyStdDev = 0.3
                + 0.4 * (estimate.avgTagDist * estimate.avgTagDist) / Math.max(1, estimate.tagCount);
            // MegaTag1 heading is trusted while seeding - tightly with 2+
            // tags so the stationary heading collapses onto the solve in a
            // few frames even at the disabled throttle's ~1 Hz - and
            // MegaTag2's heading never is.
            double rotStdDev = seedingHeading
                ? (estimate.tagCount >= 2
                    ? VisionConstants.MT1_MULTI_TAG_ROTATION_STD_DEV
                    : VisionConstants.MT1_SINGLE_TAG_ROTATION_STD_DEV)
                : 9999999;

            swerve.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

            lastFusedPose[camera] = estimate.pose;
            lastFusedTime[camera] = now;
            if (seedingHeading) {
                lastHeadingSeedTime = now;
                if (estimate.tagCount >= 2) {
                    lastStrongHeadingSeedTime = now;
                    lastStrongSeedHeading = estimate.pose.getRotation();
                }
            }
        }

        LimelightHelpers.Flush();
    }

    @Override
    public void periodic() {
        // Full-rate processing only while enabled or in Test mode (thermal / fan noise)
        updateThrottle();

        // One pose snapshot for this loop (getState() is the shared object
        // the odometry thread rewrites)
        Pose2d robotPose = swerve.getStateCopy().Pose;

        // Refresh the target caches once per loop; all readers use these.
        refreshTargets(robotPose);
        refreshSeenTags();

        if (positionTrackingEnabled) {
            updateRobotPosition(robotPose);
        }
    }
}
