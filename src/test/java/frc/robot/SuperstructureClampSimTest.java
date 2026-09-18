package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;

/**
 * The two clamps together: the carriage limited by where the arm is
 * (ceilingForSweep / floorForSweep) and the arm limited by where the
 * carriage is (armLimitForHeight). Steps both mechanisms through the moves
 * that use them, at several relative speeds and with the carriage stalling
 * mid-move, and requires that the pose never leaves the clearance table,
 * the robot-proven L4 pre-top rule holds, and the move still finishes
 * (neither clamp can wait on the other). Pure logic - no HAL. The
 * sequencing here mirrors Superstructure.approach / leaveHighPose; the
 * geometry functions are the real ones.
 */
class SuperstructureClampSimTest {

    private static final double DT = 0.005;           // physics step, seconds
    private static final int LOOP_EVERY = 4;          // robot loop = 20 ms
    private static final double RAISE = PivotPresetAngles.RAISE.getAngle();
    private static final double MARGIN = SuperstructureConstants.RATCHET_MARGIN;

    /** Acceleration-limited axis that always brakes in time for its setpoint (no overshoot). */
    private static final class Axis {
        double position, velocity, setpoint, maxVelocity;
        final double maxAcceleration;

        Axis(double position, double velocity, double maxVelocity, double maxAcceleration) {
            this.position = position;
            this.velocity = velocity;
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

    private interface Logic {
        /** One robot loop. Returns true when the move is complete. */
        boolean loop(Axis carriage, Axis arm);
    }

    /** Runs a move; returns the slowest the carriage went while strictly between the two heights. */
    private static double run(String name, Axis carriage, Axis arm, Logic logic, double timeLimit,
            double stallFrom, double stallFor, double watchFrom, double watchTo) {
        double cruise = carriage.maxVelocity;
        double slowest = Double.MAX_VALUE;
        boolean done = false;
        int steps = (int) (timeLimit / DT);
        for (int i = 0; i < steps && !done; i++) {
            double t = i * DT;
            carriage.maxVelocity = (t >= stallFrom && t < stallFrom + stallFor) ? 0.0 : cruise;
            if (i % LOOP_EVERY == 0) {
                done = logic.loop(carriage, arm);
            }
            carriage.step();
            arm.step();
            assertTrue(Superstructure.poseClear(carriage.position, arm.position),
                String.format("%s: left the clearance table at t=%.3f s (%.2f in, %.1f deg)",
                    name, t, carriage.position, arm.position));
            assertTrue(carriage.position <= SuperstructureConstants.L4_PRE_TOP_HEIGHT + 0.05
                    || arm.position <= SuperstructureConstants.L4_FINAL_GATE_ANGLE + 0.05,
                String.format("%s: above the pre-top height with the arm at %.1f deg (t=%.3f s, %.2f in)",
                    name, arm.position, t, carriage.position));
            if (carriage.position > watchFrom && carriage.position < watchTo) {
                slowest = Math.min(slowest, Math.abs(carriage.velocity));
            }
        }
        assertTrue(done, String.format("%s: did not finish in %.1f s (stuck at %.2f in, %.1f deg)",
            name, timeLimit, carriage.position, arm.position));
        return slowest;
    }

    private static double climbGoal(double target, double armAngle, double armDestination, double height) {
        double raw = Superstructure.ceilingForSweep(armAngle, armDestination, height);
        return target <= raw ? target : raw - MARGIN;
    }

    private static double descentGoal(double target, double armAngle, double armDestination, double height) {
        double raw = Superstructure.floorForSweep(armAngle, armDestination, height);
        return target >= raw ? target : raw + MARGIN;
    }

    // ------------------------------------------------------------------
    // The moves (mirrors of Superstructure.approach / leaveHighPose)
    // ------------------------------------------------------------------

    /** approach(L4): arm released at the release height, clamped by height; carriage behind the ratchet, pre-top gated. */
    private static Logic l4Climb() {
        double l4 = PresetHeights.CORAL_L4.getHeight();
        double a4 = PivotPresetAngles.CORAL_L4.getAngle();
        boolean[] released = {false};
        return (carriage, arm) -> {
            double h = carriage.position;
            if (!released[0] && h >= SuperstructureConstants.ARM_RELEASE_MIN_HEIGHT
                    && h <= SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT) {
                released[0] = true;
            }
            if (released[0]) {
                double limit = Superstructure.armLimitForHeight(arm.position, a4, h);
                if (h < SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT) {
                    limit = Math.max(limit, SuperstructureConstants.L4_STAGE_ANGLE);
                }
                arm.setpoint = limit;
            }
            double target = !released[0]
                ? SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT - MARGIN
                : arm.position <= SuperstructureConstants.L4_FINAL_GATE_ANGLE
                    ? l4 : SuperstructureConstants.L4_PRE_TOP_HEIGHT;
            double armDestination = released[0] ? a4 : RAISE;
            carriage.setpoint = Math.max(climbGoal(target, arm.position, armDestination, h), carriage.setpoint);
            return Math.abs(h - l4) < 0.05 && Math.abs(arm.position - a4) < 0.5;
        };
    }

    /** approach(L3): carriage straight to the target at RAISE, arm released at the release height. */
    private static Logic l3Climb() {
        double l3 = PresetHeights.CORAL_L3.getHeight();
        double a3 = PivotPresetAngles.CORAL_L3.getAngle();
        boolean[] released = {false};
        return (carriage, arm) -> {
            double h = carriage.position;
            carriage.setpoint = l3;
            if (!released[0] && h >= SuperstructureConstants.ARM_RELEASE_MIN_HEIGHT
                    && h <= SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT) {
                released[0] = true;
            }
            if (released[0]) {
                arm.setpoint = Superstructure.armLimitForHeight(arm.position, a3, h);
            }
            return Math.abs(h - l3) < 0.05 && Math.abs(arm.position - a3) < 0.5;
        };
    }

    /** leaveHighPose toward a low target: drop, station, then down, with the arm clamped by height on its way to RAISE. */
    private static Logic l4Exit() {
        boolean[] released = {false};
        return (carriage, arm) -> {
            double h = carriage.position;
            if (!released[0] && h <= SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT) {
                released[0] = true;
            }
            if (released[0]) {
                arm.setpoint = Superstructure.armLimitForHeight(arm.position, RAISE, h);
            }
            double tol = SuperstructureConstants.SAFE_ANGLE_TOLERANCE;
            double target = arm.position >= SuperstructureConstants.BAND_PASS_MIN_ANGLE - tol
                ? ElevatorConstants.ELEVATOR_ZERO_HEIGHT
                : arm.position >= SuperstructureConstants.L4_RETURN_STAGE_DONE_ANGLE - tol
                    ? SuperstructureConstants.L4_STATION_HEIGHT
                    : SuperstructureConstants.L4_RETURN_DROP_HEIGHT;
            carriage.setpoint = Math.min(descentGoal(target, arm.position, RAISE, h), carriage.setpoint);
            // leaveHighPose hands over once the arm has cleared band A
            return arm.position >= SuperstructureConstants.BAND_PASS_MIN_ANGLE - tol
                && h <= SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT;
        };
    }

    private static Axis carriage(double height, double velocity, double speedScale) {
        return new Axis(height, velocity * speedScale, ElevatorConstants.ELEVATOR_MAX_VELOCITY * speedScale,
            ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
    }

    private static Axis arm(double angle, double speedScale) {
        return new Axis(angle, 0.0, Constants.CorAlConstants.CORAL_PIVOT_MAX_VELOCITY * speedScale,
            Constants.CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION * speedScale);
    }

    // ------------------------------------------------------------------

    @Test
    void l4ClimbStaysInsideAndFinishesAtAnyRelativeSpeed() {
        // Handed over from the low-box escape: carriage passing the roof at speed, arm at RAISE
        double nominal = run("L4 climb", carriage(16.0, 40.0, 1.0), arm(RAISE, 1.0), l4Climb(), 4.0, 9, 0, 26.0, 46.0);
        run("L4 climb, arm at half speed", carriage(16.0, 40.0, 1.0), arm(RAISE, 0.5), l4Climb(), 6.0, 9, 0, 26.0, 46.0);
        run("L4 climb, carriage at 60%", carriage(16.0, 40.0, 0.6), arm(RAISE, 1.0), l4Climb(), 6.0, 9, 0, 26.0, 46.0);
        // The carriage stalls for a second just after the arm is released: the arm must wait at a
        // row edge, not sweep on into band A, and the move must pick up again
        run("L4 climb, carriage stalls at 25 in", carriage(16.0, 40.0, 1.0), arm(RAISE, 1.0), l4Climb(), 7.0,
            0.25, 1.0, 26.0, 46.0);
        run("L4 climb, carriage stalls at 31 in", carriage(16.0, 40.0, 1.0), arm(RAISE, 1.0), l4Climb(), 7.0,
            0.40, 1.0, 26.0, 46.0);
        // Handed over with the carriage still deep in the low box (slow elevator, or L2 -> L4): the
        // carriage must not be clamped to the low-box roof for a sweep the parked arm has not begun
        run("L4 climb from 12 in", carriage(12.0, 0.0, 1.0), arm(RAISE, 1.0), l4Climb(), 5.0, 9, 0, 26.0, 46.0);
        run("L4 climb from 12 in, carriage at 60%", carriage(12.0, 0.0, 0.6), arm(RAISE, 1.0), l4Climb(), 7.0, 9, 0, 26.0, 46.0);
        // The point of releasing the arm early: the carriage no longer stops at its first clamp
        assertTrue(nominal > 8.0, "the carriage should keep moving through the L4 climb, slowest was "
            + nominal + " in/s");
    }

    @Test
    void l3ClimbStaysInsideAndFinishes() {
        run("L3 climb", carriage(16.0, 40.0, 1.0), arm(RAISE, 1.0), l3Climb(), 4.0, 9, 0, 0, 0);
        run("L3 climb, arm at half speed", carriage(16.0, 40.0, 1.0), arm(RAISE, 0.5), l3Climb(), 6.0, 9, 0, 0, 0);
        run("L3 climb, carriage stalls at 25 in", carriage(16.0, 40.0, 1.0), arm(RAISE, 1.0), l3Climb(), 7.0,
            0.25, 1.0, 0, 0);
        // Coming down to L3 from above the window (e.g. from the high algae pose at RAISE)
        run("L3 from above", carriage(37.5, 0.0, 1.0), arm(RAISE, 1.0), l3Climb(), 4.0, 9, 0, 0, 0);
    }

    @Test
    void l4ExitStaysInsideAndFinishes() {
        double l4 = PresetHeights.CORAL_L4.getHeight();
        double a4 = PivotPresetAngles.CORAL_L4.getAngle();
        run("L4 exit", carriage(l4, 0.0, 1.0), arm(a4, 1.0), l4Exit(), 4.0, 9, 0, 0, 0);
        run("L4 exit, arm at half speed", carriage(l4, 0.0, 1.0), arm(a4, 0.5), l4Exit(), 6.0, 9, 0, 0, 0);
        run("L4 exit, carriage at 60%", carriage(l4, 0.0, 0.6), arm(a4, 1.0), l4Exit(), 6.0, 9, 0, 0, 0);
        run("L4 exit, carriage stalls at 42 in", carriage(l4, 0.0, 1.0), arm(a4, 1.0), l4Exit(), 7.0, 0.45, 1.0, 0, 0);
    }

    /**
     * The clamp on its own: from every clear pose, toward every operator
     * angle, the sweep it allows is clear at that height, and it never
     * shortens a sweep that is clear all the way.
     */
    @Test
    void armLimitNeverAllowsABlockedSweep() {
        double[] targets = {0.0, 12.5, 20.0, 25.0, 100.0, 105.0, 160.0};
        for (double h = ElevatorConstants.ELEVATOR_ZERO_HEIGHT; h <= ElevatorConstants.ELEVATOR_MAX_POSITION; h += 0.25) {
            for (double a = 0.0; a <= 160.0; a += 1.0) {
                if (!Superstructure.poseClear(h, a)) {
                    continue;
                }
                for (double target : targets) {
                    double limit = Superstructure.armLimitForHeight(a, target, h);
                    assertTrue(Superstructure.pivotPathClear(a, limit, h),
                        String.format("from %.0f deg toward %.1f at %.2f in the clamp allowed %.1f", a, target, h, limit));
                }
            }
        }
        // Open all the way -> the target itself
        assertEquals(PivotPresetAngles.CORAL_L3.getAngle(),
            Superstructure.armLimitForHeight(RAISE, PivotPresetAngles.CORAL_L3.getAngle(), PresetHeights.CORAL_L3.getHeight()), 1e-9);
        assertEquals(RAISE, Superstructure.armLimitForHeight(45.0, RAISE, 33.0), 1e-9);
        // A pose outside the table holds the arm where it is (20 deg is blocked at 33 in)
        assertEquals(20.0, Superstructure.armLimitForHeight(20.0, RAISE, 33.0), 1e-9);
        // Carriage still in band A's shadow: held above the 65-70 row
        assertEquals(72.5, Superstructure.armLimitForHeight(RAISE, PivotPresetAngles.CORAL_L4.getAngle(), 20.0), 1e-9);
        // At the top of travel the arm may not leave the L4 row
        assertEquals(22.5, Superstructure.armLimitForHeight(PivotPresetAngles.CORAL_L4.getAngle(), RAISE,
            PresetHeights.CORAL_L4.getHeight()), 1e-9);
    }
}
