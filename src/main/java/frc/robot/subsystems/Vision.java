package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Superstructure;
import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem: multi-Limelight AprilTag targeting and pose estimation.
 *
 * Pose estimation uses MegaTag2: the robot's gyro heading is sent to each
 * Limelight every loop, and the returned pose estimates are fused into the
 * swerve drivetrain's odometry with distance/tag-count based confidence.
 *
 * The "best target" (closest TRACKABLE tag - reef or coral station; barge
 * and processor tags are ignored) is computed once per loop in periodic()
 * and cached, so dashboard widgets and the tracking command can read it
 * without re-querying NetworkTables dozens of times per cycle.
 *
 * Tracking goals depend on what the robot is doing, not just which tag it
 * sees: the resolver combines the tag's class (reef / coral station) with
 * the superstructure's current goal (L1 standoff vs. L2-L4 flush-on-branch
 * vs. algae flush-centered) and the driver-selected branch side.
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

    /** A resolved alignment goal in camera space (meters). */
    public static class TrackingGoal {
        /** Desired camera-space X of the tag (+ = tag to the robot's right). */
        public final double lateral;
        /** Desired camera-space Z distance to the tag. */
        public final double distance;

        public TrackingGoal(double lateral, double distance) {
            this.lateral = lateral;
            this.distance = distance;
        }
    }

    private final Swerve swerve;

    private boolean trackingEnabled = false;
    private boolean positionTrackingEnabled = true;

    /** Driver-selected reef branch for L2-L4 alignment. */
    private BranchSide branchSide = BranchSide.LEFT;

    /** Where the superstructure goal comes from (wired in RobotContainer). */
    private Supplier<Superstructure.Goal> goalSupplier = () -> Superstructure.Goal.STOW;

    // Best target cache, refreshed once per periodic()
    private Optional<AprilTagTarget> cachedBestTarget = Optional.empty();

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            // Set all Limelights to the AprilTag pipeline
            LimelightHelpers.setPipelineIndex("limelight-" + name, VisionConstants.APRILTAG_PIPELINE);
            // Turn off Limelight LEDs during initialization
            LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
        }
    }

    /** Whether the given Limelight (by short name) currently sees a target. */
    public boolean hasTarget(String name) {
        return LimelightHelpers.getTV("limelight-" + name);
    }

    // Toggles AprilTag tracking and controls Limelight LEDs.
    public void toggleTracking(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            for (String name : VisionConstants.LIMELIGHT_NAMES) {
                if (trackingEnabled) {
                    LimelightHelpers.setLEDMode_ForceOn("limelight-" + name);
                } else {
                    LimelightHelpers.setLEDMode_ForceOff("limelight-" + name);
                }
            }
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

    private static boolean isReefTag(int tagId) {
        for (int tag : VisionConstants.REEF_TAGS) {
            if (tag == tagId) {
                return true;
            }
        }
        return false;
    }

    private static boolean isCoralStationTag(int tagId) {
        for (int tag : VisionConstants.CORAL_STATION_TAGS) {
            if (tag == tagId) {
                return true;
            }
        }
        return false;
    }

    /**
     * Resolves the alignment goal for a tag, combining the tag's class with
     * the superstructure's current goal and the selected branch side.
     * Returns empty for tags that are intentionally not tracked (barge and
     * processor) - the tracker treats those like no target at all.
     *
     * Reef goals:
     *  - CORAL_L1: centered, held L1_SCORE_DISTANCE away so the arm can
     *    swing to its 100-degree pose without hitting the reef.
     *  - CORAL_L2/L3/L4: bumpers flush, laterally centered on the selected
     *    branch. Moving the ROBOT left makes the tag appear further RIGHT
     *    in the camera, so the LEFT branch means a POSITIVE desired camera
     *    X (VERIFY the sign convention on the robot).
     *  - Everything else (algae intakes, stow, ...): bumpers flush, centered.
     *
     * Coral station goals: rear bumpers flush with the wall, centered
     * (approached backward via the rear-facing camera).
     */
    public Optional<TrackingGoal> getTrackingGoal(int tagId) {
        if (isReefTag(tagId)) {
            switch (goalSupplier.get()) {
                case CORAL_L1:
                    return Optional.of(new TrackingGoal(0.0, VisionConstants.L1_SCORE_DISTANCE));
                case CORAL_L2:
                case CORAL_L3:
                case CORAL_L4:
                    double lateral = branchSide == BranchSide.LEFT
                        ? VisionConstants.REEF_BRANCH_OFFSET
                        : -VisionConstants.REEF_BRANCH_OFFSET;
                    return Optional.of(new TrackingGoal(lateral, VisionConstants.REEF_FLUSH_DISTANCE));
                default:
                    return Optional.of(new TrackingGoal(0.0, VisionConstants.REEF_FLUSH_DISTANCE));
            }
        }
        if (isCoralStationTag(tagId)) {
            return Optional.of(new TrackingGoal(0.0, VisionConstants.STATION_FLUSH_DISTANCE));
        }
        // Barge (4, 5, 14, 15) and processor (3, 16): intentionally blank
        return Optional.empty();
    }

    /**
     * The best (closest) AprilTag target seen this loop (cached; refreshed
     * in periodic).
     */
    public Optional<AprilTagTarget> getBestTarget() {
        return cachedBestTarget;
    }

    // Queries every Limelight and picks the CLOSEST visible TRACKABLE tag
    // (smallest ground distance in camera space). Tags with no tracking
    // goal (barge/processor) are skipped entirely.
    private Optional<AprilTagTarget> findBestTarget() {
        AprilTagTarget bestTarget = null;
        double bestDistance = Double.MAX_VALUE;

        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            String name = VisionConstants.LIMELIGHT_NAMES[i];
            String limelightName = "limelight-" + name;
            if (!LimelightHelpers.getTV(limelightName)) {
                continue;
            }

            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            if (getTrackingGoal(tagId).isEmpty()) {
                continue; // Intentionally untracked tag class
            }

            // Limelight camera space: X = right, Y = down, Z = forward.
            // Ground distance ignores the vertical (Y) axis.
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
            double distance = Math.hypot(targetPose.getX(), targetPose.getZ());
            if (distance < bestDistance) {
                bestTarget = new AprilTagTarget(
                    tagId,
                    LimelightHelpers.getTX(limelightName),
                    LimelightHelpers.getTY(limelightName),
                    targetPose.getX(),
                    targetPose.getY(),
                    targetPose.getZ(),
                    name,
                    VisionConstants.LIMELIGHT_FACING_SIGNS[i]
                );
                bestDistance = distance;
            }
        }

        return Optional.ofNullable(bestTarget);
    }

    /**
     * Enables or disables MegaTag2 pose fusion. Useful during testing to
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
     * An AprilTag target detected by a Limelight, in camera space:
     * poseX = right (m), poseY = down (m), poseZ = forward (m).
     * Driving math uses X (lateral) and Z (distance); Y is display-only.
     */
    public static class AprilTagTarget {
        public final int id;
        public final double tx; // Horizontal angle to target (degrees, + = right)
        public final double ty; // Vertical angle to target (degrees)
        public final double poseX;
        public final double poseY;
        public final double poseZ;
        public final String limelightName;
        /** +1 if the reporting camera faces the robot's front, -1 if the rear. */
        public final double facingSign;

        public AprilTagTarget(int id, double tx, double ty,
                double poseX, double poseY, double poseZ, String limelightName,
                double facingSign) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.poseX = poseX;
            this.poseY = poseY;
            this.poseZ = poseZ;
            this.limelightName = limelightName;
            this.facingSign = facingSign;
        }

        /** Ground-plane distance from the camera to the tag, in meters. */
        public double groundDistance() {
            return Math.hypot(poseX, poseZ);
        }
    }

    /**
     * Fuses MegaTag2 pose estimates from every Limelight into the drivetrain
     * odometry. MegaTag2 needs the robot's current heading, which is sent to
     * each camera first; in return it produces a much more stable pose than
     * single-tag solves. Timestamp conversion to the Phoenix timebase happens
     * inside Swerve.addVisionMeasurement().
     */
    private void updateRobotPosition() {
        double headingDegrees = swerve.getState().Pose.getRotation().getDegrees();

        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            String limelightName = "limelight-" + name;

            LimelightHelpers.SetRobotOrientation(limelightName, headingDegrees, 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }

            // Confidence scales with tag count and closeness. MegaTag2 heading
            // comes from our own gyro, so give it effectively no weight.
            double xyStdDev = 0.3
                + 0.4 * (estimate.avgTagDist * estimate.avgTagDist) / Math.max(1, estimate.tagCount);

            swerve.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        }
    }

    @Override
    public void periodic() {
        // Refresh the best-target cache once per loop; all readers use this.
        cachedBestTarget = findBestTarget();

        if (positionTrackingEnabled) {
            updateRobotPosition();
        }
    }
}
