package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraPose;
import frc.robot.Constants.VisionConstants.TagClass;
import frc.robot.subsystems.Vision;

/**
 * Pins the camera-space -> robot-frame conversion the alignment tracker
 * relies on, and the field-layout headings it squares up to, against the
 * camera poses in Constants and the rotations the authored paths end at.
 * Pure geometry - no HAL.
 */
class VisionGeometryTest {

    private static final double EPS = 1e-3;

    @Test
    void unpitchedForwardCameraAtCenterIsIdentity() {
        CameraPose cam = new CameraPose(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, true);
        Translation2d t = Vision.tagPositionInRobotFrame(cam, 0.5, 0.0, 2.0);
        assertEquals(2.0, t.getX(), EPS, "camera Z (forward) becomes robot X");
        assertEquals(-0.5, t.getY(), EPS, "camera X (right) becomes robot -Y");
    }

    @Test
    void reefCameraOffsetYawAndPitchArePlacedOnTheRobot() {
        // reef: 11.04 in forward, 11.25 in LEFT, pitched 20 deg down, yawed 30 deg to the right
        CameraPose cam = VisionConstants.LIMELIGHT_POSES[2];
        // A tag on the optical axis 2 m away lies 2 cos(20) along the camera heading
        double along = 2.0 * Math.cos(Math.toRadians(20.0));
        Translation2d expected = new Translation2d(11.04 * 0.0254, 11.25 * 0.0254)
            .plus(new Translation2d(along, 0.0).rotateBy(Rotation2d.fromDegrees(-30.0)));
        Translation2d t = Vision.tagPositionInRobotFrame(cam, 0.0, 0.0, 2.0);
        assertEquals(expected.getX(), t.getX(), EPS);
        assertEquals(expected.getY(), t.getY(), EPS);
        assertTrue(t.getY() < 0.0, "an on-axis tag is to the robot's RIGHT of center for a right-yawed camera");
    }

    @Test
    void rearFunnelCameraPutsTheTagBehindTheRobot() {
        // funnel: 14 in behind center, pitched 50 deg up, facing straight back
        CameraPose cam = VisionConstants.LIMELIGHT_POSES[0];
        Translation2d t = Vision.tagPositionInRobotFrame(cam, 0.0, 0.0, 1.0);
        assertEquals(-(14.0 * 0.0254 + Math.cos(Math.toRadians(50.0))), t.getX(), EPS);
        assertEquals(0.0, t.getY(), EPS);
        // Image-down (camera +Y) points along-the-heading-and-down for an
        // up-tilted camera (the bottom of the image looks toward the
        // horizon), so a point lower in the image at the same depth is
        // further out: behind a rear camera, further behind the robot
        Translation2d lower = Vision.tagPositionInRobotFrame(cam, 0.0, 0.5, 1.0);
        assertTrue(lower.getX() < t.getX(), "a point lower in the image is further behind the up-tilted rear camera");
    }

    @Test
    void squareHeadingsMatchTheAuthoredPathEndRotations() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);
        // Blue reef face nearest the blue wall (tag 18 faces -X): the robot faces +X to be square
        assertEquals(0.0, Vision.squareHeading(layout, 18, TagClass.REEF).orElseThrow().getDegrees(), 0.5);
        // Front-left reef face (tag 19): the "Front Left Reef" paths end at -60 deg
        assertEquals(-60.0, Vision.squareHeading(layout, 19, TagClass.REEF).orElseThrow().getDegrees(), 0.5);
        // Coral stations are backed into: the station paths end at 54 (right, tag 12) and -54 (left, tag 13)
        assertEquals(54.0, Vision.squareHeading(layout, 12, TagClass.CORAL_STATION).orElseThrow().getDegrees(), 0.5);
        assertEquals(-54.0, Vision.squareHeading(layout, 13, TagClass.CORAL_STATION).orElseThrow().getDegrees(), 0.5);
        assertEquals(Optional.empty(), Vision.squareHeading(layout, 99, TagClass.REEF));
        // The layout is the 2025 field the flip constants describe
        assertEquals(VisionConstants.FIELD_LENGTH_METERS, layout.getFieldLength(), 1e-3);
        assertEquals(VisionConstants.FIELD_WIDTH_METERS, layout.getFieldWidth(), 1e-3);
    }

    @Test
    void fieldAndRobotFrameRoundTripThroughTheRobotPose() {
        // Robot at (1, 2) facing +Y (90 deg): a tag 1 m straight ahead is at field (1, 3)
        Pose2d robot = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0));
        Translation2d ahead = new Translation2d(1.0, 0.0);
        Translation2d onField = Vision.robotFrameToField(robot, ahead);
        assertEquals(1.0, onField.getX(), EPS);
        assertEquals(3.0, onField.getY(), EPS);
        Translation2d back = Vision.fieldToRobotFrame(robot, onField);
        assertEquals(1.0, back.getX(), EPS);
        assertEquals(0.0, back.getY(), EPS);
        // After the robot drives 0.4 m forward, the remembered tag is 0.6 m ahead
        Pose2d moved = new Pose2d(1.0, 2.4, Rotation2d.fromDegrees(90.0));
        assertEquals(0.6, Vision.fieldToRobotFrame(moved, onField).getX(), EPS);
        // A tag to the robot's LEFT (+Y) when facing +Y is at smaller field X
        Translation2d leftOnField = Vision.robotFrameToField(robot, new Translation2d(0.0, 0.5));
        assertEquals(0.5, leftOnField.getX(), EPS);
        assertEquals(2.0, leftOnField.getY(), EPS);
    }

    @Test
    void tagClassesFollowTheManual() {
        assertEquals(TagClass.REEF, Vision.classOf(21));
        assertEquals(TagClass.CORAL_STATION, Vision.classOf(1));
        assertEquals(TagClass.PROCESSOR, Vision.classOf(3), "processor");
        assertEquals(TagClass.BARGE, Vision.classOf(14), "barge");
        assertEquals(TagClass.NONE, Vision.classOf(99), "not a 2025 tag");
    }
}
