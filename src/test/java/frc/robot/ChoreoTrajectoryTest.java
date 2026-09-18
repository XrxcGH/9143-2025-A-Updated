package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.path.PathPlannerPath;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

/**
 * Proves the Choreo trajectory integration end to end: a real {@code .traj}
 * file in {@code deploy/choreo} is parsed by PathPlanner's Choreo loader
 * ({@link PathPlannerPath#fromChoreoTrajectory}), the same call
 * RobotContainer.addChoreoAutos() uses to offer Choreo trajectories alongside
 * the PathPlanner autos. If the file schema or the loader ever drifts, this
 * fails before it reaches a robot.
 */
class ChoreoTrajectoryTest {

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void teardown() {
        HAL.shutdown();
    }

    @Test
    void demoChoreoTrajectoryLoads() throws Exception {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("Demo");
        assertNotNull(path, "Choreo trajectory 'Demo' should load");
        assertTrue(path.numPoints() > 1,
            "Loaded Choreo path should have multiple points, got " + path.numPoints());
        assertTrue(path.getStartingHolonomicPose().isPresent(),
            "Choreo swerve path should expose a holonomic starting pose");
    }
}
