package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.subsystems.Elevator;

/**
 * Exercises the Elevator's desktop physics simulation: drives the carriage
 * open-loop to prove the sim wiring (motor output -> plant -> simulated
 * encoder), then verifies the stick-release hold latches on the controller.
 *
 * NOTE: this deliberately does NOT assert a full MAXMotion profile move.
 * REV's simulated MAXMotion profile advances on WALL-CLOCK time, not the
 * stepped HAL clock, so profile-following in a fast unit-test loop is
 * nondeterministic (observed landing anywhere from 20 to 43 inches for a
 * 20 inch command depending on host speed). Holding at zero profile
 * distance is unaffected. Verify profile moves interactively with
 * `./gradlew simulateJava` and on the robot.
 */
class ElevatorSimTest {

    private Elevator elevator;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        // Keep the HAL clock in lockstep with the physics steps
        SimHooks.pauseTiming();
        // The Spark MAX sim only drives outputs while the robot is enabled
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        elevator = new Elevator();
    }

    @AfterEach
    void teardown() {
        SimHooks.resumeTiming();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        HAL.shutdown();
    }

    /** Runs one 20 ms step of physics + simulated clock. */
    private void step() {
        DriverStationSim.notifyNewData();
        elevator.simulationPeriodic();
        SimHooks.stepTiming(0.02);
    }

    @Test
    void simWiringDrivesAndHoldLatches() {
        assertEquals(0.0, elevator.getCurrentPosition(), 0.5, "Elevator must start at its base");

        // 2 seconds of full manual up-stick: the simulated carriage must
        // rise against simulated gravity (proves output -> plant -> encoder)
        for (int i = 0; i < 100; i++) {
            elevator.manualControl(1.0);
            step();
        }
        assertTrue(elevator.getCurrentPosition() > 2.0,
            "Manual control should raise the simulated carriage, was "
                + elevator.getCurrentPosition());

        // Release the stick: RobotContainer's default command calls
        // holdCurrentPosition(), which must latch the height on the
        // controller so the carriage does not sink under gravity
        elevator.holdCurrentPosition();
        double heldHeight = elevator.getCurrentPosition();

        for (int i = 0; i < 250; i++) {
            step();
        }

        assertEquals(heldHeight, elevator.getCurrentPosition(), 2.0,
            "Carriage should hold its height after the stick is released");
    }
}
