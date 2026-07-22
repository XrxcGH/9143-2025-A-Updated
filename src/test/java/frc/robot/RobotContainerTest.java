package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Smoke test: the entire robot wires up (every subsystem constructs, all
 * bindings resolve, the PathPlanner AutoBuilder configures from the deploy
 * directory) and the command scheduler runs. Catches broken CAN ID maps,
 * missing deploy files, and binding conflicts without a robot.
 */
class RobotContainerTest {

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    @Test
    void robotWiresUpAndSchedulerRuns() {
        assertDoesNotThrow(() -> {
            new RobotContainer();
            for (int i = 0; i < 5; i++) {
                CommandScheduler.getInstance().run();
            }
        });
    }
}
