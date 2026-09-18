package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.CorAl;
import frc.robot.subsystems.Elevator;

/**
 * Binding-logic checks for the operator controller mapping (see the control
 * map at the top of RobotContainer). Nothing here
 * depends on simulated motion (REV's simulated MAXMotion runs on wall-clock
 * time): it presses simulated buttons and looks at what the scheduler did.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class ControlsBindingTest {

    private RobotContainer container;
    private Elevator elevator;
    private CorAl coral;
    private Superstructure superstructure;
    private XboxControllerSim operator;
    private final List<String> interrupted = new ArrayList<>();

    private XboxControllerSim driver;

    /** One container per JVM: the Spark MAX ids can only be claimed once. */
    @BeforeAll
    void setup() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        operator = new XboxControllerSim(1);
        driver = new XboxControllerSim(0);
        neutral();
        container = new RobotContainer();
        elevator = (Elevator) field("elevator");
        coral = (CorAl) field("coral");
        superstructure = (Superstructure) field("superstructure");
        CommandScheduler.getInstance().onCommandInterrupt(c -> interrupted.add(c.getName()));
    }

    /** Everything released. The simulated POV reads 0 (= UP) until it is set, so centre it. */
    private void neutral() {
        for (XboxControllerSim pad : new XboxControllerSim[] {driver, operator}) {
            for (int b = 1; b <= 10; b++) {
                pad.setRawButton(b, false);
            }
            for (int a = 0; a < 6; a++) {
                pad.setRawAxis(a, 0.0);
            }
            pad.setPOV(-1);
            pad.notifyNewData();
        }
    }

    @BeforeEach
    void reset() {
        neutral();
        run(2);
        CommandScheduler.getInstance().cancelAll();
        run(2);
        interrupted.clear();
    }

    @AfterAll
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        HAL.shutdown();
    }

    private Object field(String name) throws Exception {
        Field f = RobotContainer.class.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(container);
    }

    private void run(int loops) {
        for (int i = 0; i < loops; i++) {
            operator.notifyNewData();
            driver.notifyNewData();
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    @Test
    void sticksAreDeadWithoutTheManualModifier() {
        operator.setLeftY(-1.0);   // full up
        operator.setRightY(-1.0);
        run(10);
        assertFalse(elevator.isInManualMode(), "stick alone must not drive the elevator");
        assertEquals(null, elevator.getCurrentCommand(), "no default command owns the elevator");
    }

    @Test
    void manualModifierTakesOverAndReleases() {
        operator.setLeftBumperButton(true);
        operator.setLeftY(-1.0);
        run(5);
        assertTrue(elevator.isInManualMode(), "LB + stick = manual elevator");
        operator.setLeftY(0.0);
        operator.setLeftBumperButton(false);
        run(5);
        assertFalse(elevator.isInManualMode(), "releasing LB must hand back to closed-loop hold");
        assertEquals(null, elevator.getCurrentCommand());
    }

    @Test
    void earlyScorePullDoesNotInterruptTheMove() {
        operator.setYButton(true);          // L4
        run(2);
        operator.setYButton(false);
        run(2);
        assertEquals(Superstructure.Goal.CORAL_L4, superstructure.getGoal());
        Command move = elevator.getCurrentCommand();
        assertTrue(move != null, "the L4 move owns the elevator");
        interrupted.clear();

        operator.setRightTriggerAxis(1.0);  // pulled long before the pose is reached
        run(10);
        assertFalse(superstructure.readyToScore(), "pose not reached in this test");
        assertTrue(interrupted.isEmpty(), "an early score pull must not cancel the staged move: " + interrupted);
        assertEquals(move, elevator.getCurrentCommand());
        assertEquals(0.0, coral.getIntakeOutput(), 1e-9, "rollers must not run before the pose is reached");
    }

    @Test
    void manualModifierCancelsARunningMove() {
        operator.setBButton(true);          // L3
        run(2);
        operator.setBButton(false);
        run(2);
        assertTrue(elevator.getCurrentCommand() != null);
        interrupted.clear();

        operator.setLeftBumperButton(true); // take over
        run(2);
        assertEquals(1, interrupted.size(), "LB must cancel the running sequence: " + interrupted);
        // and presets are locked out while LB is held
        operator.setAButton(true);
        run(2);
        assertEquals(Superstructure.Goal.CORAL_L3, superstructure.getGoal(), "presets are ignored while LB is held");
    }

    @Test
    void scoreWithNoScoringGoalDoesNothing() {
        operator.setLeftTriggerAxis(1.0);   // HOME -> goal STOW
        run(2);
        operator.setLeftTriggerAxis(0.0);
        run(2);
        CommandScheduler.getInstance().cancelAll();
        run(1);
        assertEquals(Superstructure.Goal.STOW, superstructure.getGoal());
        operator.setRightTriggerAxis(1.0);
        run(5);
        assertEquals(null, coral.getCurrentCommand(), "no score command may start from the stow goal");
        assertEquals(0.0, coral.getIntakeOutput(), 1e-9);
    }

    /** The WPILib semantics the gated score relies on: (held AND ready).onTrue fires on ready's rising edge, once. */
    @Test
    void gatedTriggerFiresWhenTheGateOpens() {
        AtomicBoolean held = new AtomicBoolean(false);
        AtomicBoolean ready = new AtomicBoolean(false);
        AtomicInteger fired = new AtomicInteger();
        new Trigger(held::get).and(ready::get).onTrue(Commands.runOnce(fired::incrementAndGet));

        held.set(true);
        run(5);
        assertEquals(0, fired.get(), "held but not ready: nothing");
        ready.set(true);
        run(5);
        assertEquals(1, fired.get(), "fires the loop the gate opens, exactly once");
        ready.set(false);
        run(2);
        ready.set(true);
        run(2);
        assertEquals(2, fired.get(), "a new rising edge (next pose reached with the trigger still held) fires again");
    }
}
