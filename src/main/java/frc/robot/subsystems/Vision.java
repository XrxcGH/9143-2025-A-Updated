package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
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
 * DISABLED (and for a short window after the driver asks for a heading
 * re-seed) MegaTag1 is fused instead, because its solve carries an absolute
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
 * the robot is square to the tag's face. Cameras only report the tag class
 * they are mounted for (rear funnel camera: coral stations; reef camera:
 * reef). Once tracking starts the first chosen tag is latched so the goal
 * cannot flip between adjacent reef faces mid-approach.
 *
 * The tracker never drives on a raw camera solve. A tag does not move, so
 * each NEW camera frame is turned into a FIELD position for the latched tag
 * - using the robot's pose at the moment the image was captured, not now -
 * and low-pass filtered there; every loop the tracker then gets that field
 * position seen from the robot's CURRENT odometry pose. Filtering a static
 * point adds no lag to the control, the 50-100 ms camera latency drops out
 * (it used to show up as a phantom lateral error whenever the robot was
 * turning: range x the heading change during the latency), single-frame
 * solve noise no longer reaches the wheels, and a dropped frame changes
 * nothing - which is also how the approach finishes when the latched tag
 * leaves the camera's view (the rear camera loses the station tag before
 * the bumpers are flush).
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

    /** Where the superstructure goal comes from (wired in RobotContainer). */
    private Supplier<Superstructure.Goal> goalSupplier = () -> Superstructure.Goal.STOW;

    // Target caches, refreshed once per periodic()
    private Optional<AprilTagTarget> cachedBestTarget = Optional.empty();   // matches the current goal (drives the tracker)
    private Optional<AprilTagTarget> cachedBestVisible = Optional.empty();  // any trackable tag (dashboard / tuning)

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
        return TagClass.NONE;
    }

    // ------------------------------------------------------------------
    // Throttle
    // ------------------------------------------------------------------

    /**
     * Frame throttle currently applied to the cameras (-1 = not yet sent).
     * Tracked so the NT write happens only on enable/disable transitions.
     */
    private int appliedThrottle = -1;

    /**
     * Throttles AprilTag processing while the robot is disabled and restores
     * full rate when enabled - the cameras spend most of their powered-on
     * life disabled in the pit, where full-rate processing only makes heat.
     */
    private void updateThrottle() {
        int throttle = DriverStation.isDisabled()
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
     * touch the LEDs: they add nothing to AprilTag detection and were the
     * main reason the cameras ran hot (and loud) whenever tracking was on.
     * Turning tracking off releases the tag latch.
     */
    public void toggleTracking(boolean enabled) {
        trackingEnabled = enabled;
        if (!enabled) {
            releaseLatch();
        }
    }

    // Returns whether AprilTag tracking is enabled.
    // NOTE: intentionally unused - kept as the getter paired with
    // toggleTracking() for dashboards/tests (Swerve tracks its own copy of
    // this state for the button toggle).
    public boolean isTrackingEnabled() {
        return trackingEnabled;
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
     *    robot's RIGHT (negative left).
     *  - Everything else (algae intakes, stow, ...): bumpers flush, centered.
     *
     * Coral station goals: rear bumpers flush with the wall, centered
     * (approached backward, so the tag sits BEHIND the robot center).
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
                        return Optional.of(new TrackingGoal(Tunables.reefFlushDistance(), left));
                    default:
                        return Optional.of(new TrackingGoal(Tunables.reefFlushDistance(), 0.0));
                }
            case CORAL_STATION:
                return Optional.of(new TrackingGoal(-Tunables.stationFlushDistance(), 0.0));
            default:
                return Optional.empty();
        }
    }

    /** The target the tracker should drive to this loop (cached; refreshed in periodic). */
    public Optional<AprilTagTarget> getBestTarget() {
        return cachedBestTarget;
    }

    /**
     * The closest trackable tag of ANY class seen this loop, regardless of
     * the superstructure goal or the latch - for the dashboard and for
     * measuring the flush distances with the robot pushed into position.
     */
    public Optional<AprilTagTarget> getBestVisibleTarget() {
        return cachedBestVisible;
    }

    /**
     * True when the tag's class matches what the superstructure is doing:
     * a STOW goal means the robot is heading to a coral station to intake,
     * so only station tags are candidates; every scoring/algae goal targets
     * the reef, so only reef tags are candidates. Without this, a station
     * tag seen by the rear camera at 3 m could out-"close" the intended
     * reef tag at 4 m and hijack a scoring alignment.
     */
    private boolean tagMatchesCurrentGoal(TagClass tagClass) {
        if (goalSupplier.get() == Superstructure.Goal.STOW) {
            return tagClass == TagClass.CORAL_STATION;
        }
        return tagClass == TagClass.REEF;
    }

    private void latch(AprilTagTarget target, long sampleStamp, double captureTime, Pose2d robotPose) {
        latchedTagId = target.id;
        latchedTagClass = target.tagClass;
        latchedSquareHeading = target.squareHeading;
        latchedTagFieldPosition = null;
        latchedSampleStamp = Long.MIN_VALUE;
        latchedOutlierCount = 0;
        updateLatchedEstimate(target, sampleStamp, captureTime, robotPose);
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
    private void updateLatchedEstimate(AprilTagTarget target, long sampleStamp, double captureTime, Pose2d robotPose) {
        latchedLastSeenTime = Timer.getFPGATimestamp();
        if (sampleStamp == latchedSampleStamp) {
            return;
        }
        latchedSampleStamp = sampleStamp;
        Pose2d poseAtCapture = swerve.samplePoseAt(Utils.fpgaToCurrentTime(captureTime)).orElse(robotPose);
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

    private void releaseLatch() {
        if (latchedTagId < 0) {
            return;
        }
        latchedTagId = -1;
        latchedTagFieldPosition = null;
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
                0.0, 0.0, 0.0, 0.0, 0.0, inRobot, latchedSquareHeading, true);
        }
        return new AprilTagTarget(live.id, live.tagClass, live.limelightName, live.cameraIndex,
            live.tx, live.ty, live.cameraX, live.cameraY, live.cameraZ, inRobot, live.squareHeading, false);
    }

    /**
     * Queries every Limelight and refreshes both target caches: the closest
     * trackable tag of any class (dashboard), and the closest tag matching
     * the current goal (tracker), honoring the latch while tracking. While
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
        boolean latchedSeen = false;

        for (int i = 0; i < limelightTableNames.length; i++) {
            TagClass allowedClass = VisionConstants.LIMELIGHT_TRACKING_CLASSES[i];
            if (allowedClass == TagClass.NONE) {
                continue; // camera not used for alignment
            }
            String limelightName = limelightTableNames[i];
            if (!LimelightHelpers.getTV(limelightName)) {
                continue;
            }
            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            TagClass tagClass = classOf(tagId);
            if (tagClass != allowedClass) {
                continue; // wrong class for this camera (or barge/processor)
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
            return;
        }
        if (latchedTagId < 0) {
            if (bestForGoal != null) {
                latch(bestForGoal, bestGoalStamp, bestGoalCaptureTime, robotPose);
            }
        } else if (latchedSeen) {
            updateLatchedEstimate(bestForGoal, bestGoalStamp, bestGoalCaptureTime, robotPose);
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
     * HEADING_RESEED_WINDOW_SECONDS even though the robot is enabled. This
     * is the driver's "fix my field-centric heading" while a tag is in view;
     * unlike a gyro re-zero it cannot feed MegaTag2 a made-up heading.
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
        // Full-rate processing only while enabled (thermal / fan noise)
        updateThrottle();

        // One pose snapshot for this loop (getState() is the shared object
        // the odometry thread rewrites)
        Pose2d robotPose = swerve.getStateCopy().Pose;

        // Refresh the target caches once per loop; all readers use these.
        refreshTargets(robotPose);

        if (positionTrackingEnabled) {
            updateRobotPosition(robotPose);
        }
    }
}
