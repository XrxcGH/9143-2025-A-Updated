package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CorAlConstants;
import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ArmAxis;
import frc.robot.subsystems.CarriageAxis;

/**
 * Runs the REAL Superstructure commands - the same objects the operator's
 * buttons schedule - through the real CommandScheduler on a stepped clock,
 * against a simulated carriage and arm, for every ordered pair of operator
 * poses (and for a second button pressed mid-move). Every 5 ms it requires
 * that the pose is inside the clearance table and the L4 pre-top rule holds;
 * per move it requires that the command finishes at its pose, that the
 * carriage's setpoints do not alternate (the fault that shook the elevator
 * on the L4 exit), that neither controller is flooded with setpoints, and
 * that the two mechanisms are never both stationary mid-move.
 *
 * The vendor simulators advance on wall-clock time, so the real subsystems
 * cannot be stepped; the Superstructure therefore talks to the CarriageAxis /
 * ArmAxis interfaces, and these are the simulated implementations.
 * A metrics table for every move is written to build/sim/sequence_metrics.csv.
 */
class SuperstructureSequenceSimTest {

    private static final double DT = 0.005;
    private static final int PHYSICS_PER_LOOP = 4;

    // ------------------------------------------------------------------
    // Simulated mechanisms
    // ------------------------------------------------------------------

    /** Acceleration-limited axis that brakes in time for its setpoint, continuous across retargets. */
    private static class Profile {
        double position, velocity, setpoint;
        final double maxVelocity, maxAcceleration;

        Profile(double position, double maxVelocity, double maxAcceleration) {
            this.position = position;
            this.setpoint = position;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }

        void step() {
            double error = setpoint - position;
            double want = Math.copySign(Math.min(maxVelocity, Math.sqrt(2 * maxAcceleration * Math.abs(error))), error);
            double dv = maxAcceleration * DT;
            velocity += Math.max(-dv, Math.min(dv, want - velocity));
            position += velocity * DT;
        }
    }

    private static final class SimCarriage extends SubsystemBase implements CarriageAxis {
        final Profile axis;
        final List<Double> setpoints = new ArrayList<>();
        boolean positionMode = false;

        SimCarriage(double speedScale) {
            axis = new Profile(ElevatorConstants.ELEVATOR_ZERO_HEIGHT,
                ElevatorConstants.ELEVATOR_MAX_VELOCITY * speedScale, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        }

        @Override public double getCurrentPosition() { return axis.position; }
        @Override public double getVelocity() { return axis.velocity; }
        @Override public boolean isAtTargetPosition() {
            return Math.abs(axis.setpoint - axis.position) <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR;
        }
        @Override public double maxAcceleration() { return axis.maxAcceleration; }
        @Override public boolean isInManualMode() { return false; }
        @Override public void manualControl(double stick) { }
        @Override public void holdCurrentPosition() { setPosition(axis.position); }

        @Override public void setPosition(double height) {
            height = Math.min(Math.max(height, ElevatorConstants.ELEVATOR_ZERO_HEIGHT), ElevatorConstants.ELEVATOR_MAX_POSITION);
            if (positionMode && height == axis.setpoint) {
                return; // the real Elevator ignores a repeat of its setpoint too
            }
            positionMode = true;
            axis.setpoint = height;
            setpoints.add(height);
        }
    }

    private static final class SimArm extends SubsystemBase implements ArmAxis {
        final Profile axis;
        /** Through bore minus rotor: the chain's slack as the gates see it. */
        final double slackDegrees;
        final List<Double> setpoints = new ArrayList<>();
        boolean positionMode = false;
        double target = 0.0;

        SimArm(double speedScale, double slackDegrees) {
            axis = new Profile(0.0, CorAlConstants.CORAL_PIVOT_MAX_VELOCITY * speedScale,
                CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION * speedScale);
            this.slackDegrees = slackDegrees;
        }

        /** The real arm rests toward 0 deg of where the rotor holds it below the balance angle, and away above. */
        private double slack() {
            return axis.position < CorAlConstants.CORAL_PIVOT_BALANCE_ANGLE_DEG ? -slackDegrees : slackDegrees;
        }

        @Override public double getPivotAngle() { return Math.max(0.0, axis.position + slack()); }
        @Override public double getPivotVelocity() { return axis.velocity; }
        @Override public double getTargetAngle() { return target; }
        @Override public boolean isAtTargetAngle() {
            return Math.abs(getPivotAngle() - target) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
        }
        @Override public double maxAcceleration() { return axis.maxAcceleration; }
        @Override public void manualPivotControl(double stick) { }
        @Override public void setIntakeSpeed(double dutyCycle) { }
        @Override public void stopIntake() { }
        @Override public boolean isGamePieceDetected() { return false; }

        @Override public void setPivotAngle(double angle) {
            angle = Math.min(Math.max(angle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE), CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
            if (positionMode && angle == target) {
                return;
            }
            positionMode = true;
            target = angle;
            axis.setpoint = angle;
            setpoints.add(angle);
        }
    }

    // ------------------------------------------------------------------
    // Harness
    // ------------------------------------------------------------------

    private record Pose(String name, double height, double angle, Supplier<Command> go) { }

    private SimCarriage carriage;
    private SimArm arm;
    private Superstructure superstructure;
    private final List<String> metrics = new ArrayList<>();

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        SimHooks.resumeTiming();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private void build(double carriageSpeed, double armSpeed, double slack) {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        carriage = new SimCarriage(carriageSpeed);
        arm = new SimArm(armSpeed, slack);
        superstructure = new Superstructure(carriage, arm);
    }

    private List<Pose> poses() {
        double zero = ElevatorConstants.ELEVATOR_ZERO_HEIGHT;
        return List.of(
            new Pose("BASE", zero, PivotPresetAngles.BASE.getAngle(), superstructure::stow),
            new Pose("L1", zero, PivotPresetAngles.CORAL_L1.getAngle(), superstructure::goToCoralL1),
            new Pose("L2", PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle(), superstructure::goToCoralL2),
            new Pose("L3", PresetHeights.CORAL_L3.getHeight(), PivotPresetAngles.CORAL_L3.getAngle(), superstructure::goToCoralL3),
            new Pose("L4", PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle(), superstructure::goToCoralL4),
            new Pose("ALGAE_LOW", PresetHeights.ALGAE_LOW_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle(), superstructure::intakeAlgaeLow),
            new Pose("ALGAE_HIGH", PresetHeights.ALGAE_HIGH_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle(), superstructure::intakeAlgaeHigh),
            new Pose("BARGE", PresetHeights.ALGAE_SCORE.getHeight(), PivotPresetAngles.ALGAE_SCORE.getAngle(), superstructure::goToBarge));
    }

    /** One robot loop: physics, clock, scheduler. Checks the safety invariants at every physics step. */
    private void loop(String label, double slack) {
        for (int i = 0; i < PHYSICS_PER_LOOP; i++) {
            carriage.axis.step();
            arm.axis.step();
            double h = carriage.axis.position;
            double a = arm.getPivotAngle();
            // With chain slack the resting arm reads up to `slack` outside its row - the exposure the
            // planner's own lookups allow for; without it the pose must be strictly inside the table.
            // A scoring pose may sit ON a row edge (L3 is 25.0 deg, the edge of its row), so the arm's
            // own at-target window either side of the reading counts as the same pose.
            double window = CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
            boolean clear = Superstructure.poseClear(h, a)
                || Superstructure.poseClear(h, a + window) || Superstructure.poseClear(h, a - window)
                || (slack > 0 && Superstructure.poseClear(h, Superstructure.snapIntoTable(a, h)));
            assertTrue(clear, String.format("%s: left the clearance table at %.2f in, %.1f deg", label, h, a));
            assertTrue(h <= SuperstructureConstants.L4_PRE_TOP_HEIGHT + 0.05
                    || a <= SuperstructureConstants.L4_FINAL_GATE_ANGLE + 0.05 || a >= 97.0,
                String.format("%s: above the pre-top height with the arm at %.1f deg (%.2f in)", label, a, h));
        }
        DriverStationSim.notifyNewData();
        SimHooks.stepTiming(DT * PHYSICS_PER_LOOP);
        CommandScheduler.getInstance().run();
    }

    private static int reversals(List<Double> setpoints) {
        int count = 0;
        for (int i = 2; i < setpoints.size(); i++) {
            double first = setpoints.get(i - 1) - setpoints.get(i - 2);
            double second = setpoints.get(i) - setpoints.get(i - 1);
            if (first * second < 0) {
                count++;
            }
        }
        return count;
    }

    /** Runs a command to completion and checks the per-move requirements. */
    private void run(String label, Command command, Pose destination, double timeLimit, double slack, boolean checkMotion) {
        carriage.setpoints.clear();
        arm.setpoints.clear();
        CommandScheduler.getInstance().schedule(command);
        double stationary = 0.0;
        double longestStationary = 0.0;
        double elapsed = 0.0;
        loop(label, slack); // the first run() initializes it
        while (command.isScheduled() && elapsed < timeLimit) {
            loop(label, slack);
            elapsed += DT * PHYSICS_PER_LOOP;
            boolean arrived = Math.abs(carriage.axis.position - destination.height) < 0.3
                && Math.abs(arm.axis.position - destination.angle) < 1.5;
            if (!arrived && Math.abs(carriage.axis.velocity) < 0.5 && Math.abs(arm.axis.velocity) < 3.0) {
                stationary += DT * PHYSICS_PER_LOOP;
                longestStationary = Math.max(longestStationary, stationary);
            } else {
                stationary = 0.0;
            }
        }
        // The command may end on "at target" a moment before the mechanisms have stopped: let them land
        for (int i = 0; i < 100 && (Math.abs(carriage.axis.velocity) > 0.05 || Math.abs(arm.axis.velocity) > 0.5); i++) {
            loop(label, slack);
        }
        metrics.add(String.format("%s,%.2f,%d,%d,%d,%.2f,%.2f,%.1f", label, elapsed, carriage.setpoints.size(),
            reversals(carriage.setpoints), arm.setpoints.size(), longestStationary,
            carriage.axis.position, arm.getPivotAngle())
            + "," + carriage.setpoints.toString().replace(", ", " ") + "," + arm.setpoints.toString().replace(", ", " "));

        assertTrue(!command.isScheduled(), String.format("%s: did not finish in %.1f s (stuck at %.2f in, %.1f deg; carriage -> %.2f, arm -> %.1f)",
            label, timeLimit, carriage.axis.position, arm.getPivotAngle(), carriage.axis.setpoint, arm.target));
        assertTrue(Math.abs(carriage.axis.position - destination.height) < 0.3,
            String.format("%s: ended at %.2f in, wanted %.2f", label, carriage.axis.position, destination.height));
        assertTrue(Math.abs(arm.axis.position - destination.angle) < 1.5,
            String.format("%s: ended at %.1f deg, wanted %.1f", label, arm.axis.position, destination.angle));
        if (checkMotion) {
            // An exit through the L4 station legitimately turns round once; alternation does not stop.
            assertTrue(reversals(carriage.setpoints) <= 1,
                String.format("%s: carriage setpoints alternate %s", label, carriage.setpoints));
            assertTrue(carriage.setpoints.size() <= 8,
                String.format("%s: %d carriage setpoints %s", label, carriage.setpoints.size(), carriage.setpoints));
            assertTrue(arm.setpoints.size() <= 30,
                String.format("%s: %d arm setpoints %s", label, arm.setpoints.size(), arm.setpoints));
            assertTrue(longestStationary <= 0.2,
                String.format("%s: both mechanisms stationary for %.2f s mid-move", label, longestStationary));
        }
    }

    private void everyPair(String scenario, double carriageSpeed, double armSpeed, double slack, boolean checkMotion) {
        build(carriageSpeed, armSpeed, slack);
        int count = poses().size();
        for (int from = 0; from < count; from++) {
            for (int to = 0; to < count; to++) {
                if (from == to) {
                    continue;
                }
                build(carriageSpeed, armSpeed, slack);
                Pose start = poses().get(from);
                Pose end = poses().get(to);
                if (from != 0) {
                    run(scenario + " (setup) BASE->" + start.name, start.go.get(), start, 12.0, slack, false);
                }
                run(scenario + " " + start.name + "->" + end.name, end.go.get(), end, 12.0, slack, checkMotion);
            }
        }
    }

    private void writeMetrics(String name) throws IOException {
        Path dir = Path.of("build", "sim");
        Files.createDirectories(dir);
        List<String> lines = new ArrayList<>();
        lines.add("move,seconds,carriage_setpoints,carriage_reversals,arm_setpoints,longest_both_stationary_s,end_height,end_angle,carriage_list,arm_list");
        lines.addAll(metrics);
        Files.write(dir.resolve(name), lines);
    }

    // ------------------------------------------------------------------

    @Test
    void everyTransitionNominal() throws IOException {
        try {
            everyPair("nominal", 1.0, 1.0, 0.0, true);
        } finally {
            writeMetrics("sequence_metrics.csv");
        }
    }

    @Test
    void everyTransitionWithChainSlackAndMismatchedSpeeds() throws IOException {
        try {
            everyPair("slack2", 1.0, 1.0, 2.0, false);
            everyPair("slowArm", 1.0, 0.5, 0.0, false);
            everyPair("slowCarriage", 0.6, 1.0, 0.0, false);
        } finally {
            writeMetrics("sequence_metrics_offnominal.csv");
        }
    }

    /** A second button mid-move: the new plan starts from wherever the mechanisms are, moving. */
    @Test
    void secondButtonMidMove() throws IOException {
        try {
            double[] interruptAfter = {0.3, 0.6, 0.9, 1.2, 1.6};
            build(1.0, 1.0, 0.0);
            int count = poses().size();
            for (int first = 1; first < count; first++) {
                for (int second = 0; second < count; second++) {
                    if (first == second) {
                        continue;
                    }
                    for (double after : interruptAfter) {
                        build(1.0, 1.0, 0.0);
                        Pose a = poses().get(first);
                        Pose b = poses().get(second);
                        String label = String.format("interrupt BASE->%s at %.1f s ->%s", a.name, after, b.name);
                        Command firstMove = a.go.get();
                        CommandScheduler.getInstance().schedule(firstMove);
                        for (double t = 0; t < after; t += DT * PHYSICS_PER_LOOP) {
                            loop(label, 0.0);
                        }
                        run(label, b.go.get(), b, 12.0, 0.0, false);
                    }
                }
            }
        } finally {
            writeMetrics("sequence_metrics_interrupts.csv");
        }
    }
}
