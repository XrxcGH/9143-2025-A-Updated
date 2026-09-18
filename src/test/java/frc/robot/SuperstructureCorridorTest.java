package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Superstructure.MoveTrace;

/**
 * Pins the CAD-derived free corridors and the coordinated planner to each
 * other: every preset-to-preset move is rehearsed as pure geometry (exactly
 * the clamp the robot streams, exactly the detour the planner picks) and must
 * arrive without ever leaving the corridors. If someone edits a preset or the
 * table, this fails before the robot does. No HAL.
 */
class SuperstructureCorridorTest {

    private static final double BASE = ElevatorConstants.ELEVATOR_ZERO_HEIGHT;

    /** The operator poses, as {height, angle}. */
    private static final double[][] POSES = {
        {BASE, PivotPresetAngles.BASE.getAngle()},
        {BASE, PivotPresetAngles.CORAL_L1.getAngle()},
        {PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle()},
        {PresetHeights.CORAL_L3.getHeight(), PivotPresetAngles.CORAL_L3.getAngle()},
        {PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle()},
        {PresetHeights.ALGAE_LOW_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()},
        {PresetHeights.ALGAE_HIGH_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()},
        {PresetHeights.ALGAE_SCORE.getHeight(), PivotPresetAngles.ALGAE_SCORE.getAngle()},
        // Algae hold: the arm at the safe travel angle, carriage where it was
        {PresetHeights.ALGAE_LOW_INTAKE.getHeight(), PivotPresetAngles.RAISE.getAngle()},
        {PresetHeights.ALGAE_HIGH_INTAKE.getHeight(), PivotPresetAngles.RAISE.getAngle()},
    };

    private static final String[] NAMES = {
        "BASE", "L1", "L2", "L3", "L4", "ALGAE LOW", "ALGAE HIGH", "ALGAE SCORE", "HOLD LOW", "HOLD HIGH",
    };

    @Test
    void tableIsOrderedAndWellFormed() {
        double lastAngle = -1;
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            assertEquals(5, row.length);
            assertTrue(row[0] > lastAngle, "angle rows must be increasing");
            lastAngle = row[0];
            assertTrue(row[1] <= row[2], "low corridor min <= max");
            assertEquals(Double.isNaN(row[3]), Double.isNaN(row[4]), "high corridor is both or neither");
            if (!Double.isNaN(row[3])) {
                assertTrue(row[3] <= row[4], "high corridor min <= max");
                assertTrue(row[3] > row[2], "high corridor sits above the low corridor");
            }
        }
        assertTrue(lastAngle >= 160.0, "table covers the full pivot range");
    }

    @Test
    void measuredAndObservedContactsAreOutside() {
        assertFalse(Superstructure.poseClear(12.0, 0.0), "tucked arm hits the cross bar above ~10.75 in");
        assertFalse(Superstructure.poseClear(23.0, 5.0), "5 deg hits the middle-stage tube at ~23 in");
        assertFalse(Superstructure.poseClear(52.5, 45.0), "old L4 angle hits the middle-stage tube at the top");
        assertFalse(Superstructure.poseClear(52.5, 35.0), "35 deg is no better at the top");
        assertFalse(Superstructure.poseClear(40.0, 90.0), "90 deg is not a safe travel angle above ~36 in");
        assertFalse(Superstructure.poseClear(2.0, 160.0), "algae intake angle hits the bumper near the base");
    }

    @Test
    void presetPosesAreAllInside() {
        for (int i = 0; i < POSES.length; i++) {
            assertTrue(Superstructure.poseClear(POSES[i][0], POSES[i][1]),
                NAMES[i] + " pose must be inside the corridors");
        }
        // L2 keeps its margin even when the chain lets the arm sag 2.5 deg
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L2.getHeight(),
            PivotPresetAngles.CORAL_L2.getAngle() - 2.5));
        // The safe travel angle is clear from the base to the top
        assertTrue(Superstructure.elevatorPathClear(BASE, ElevatorConstants.ELEVATOR_MAX_POSITION,
            PivotPresetAngles.RAISE.getAngle()));
        // Nothing is inside a band, so the startup preset audit is silent
        assertEquals("", Superstructure.presetAuditMessage());
    }

    @Test
    void everyPresetToPresetMoveHasARouteThatStaysInTheCorridors() {
        for (int i = 0; i < POSES.length; i++) {
            for (int j = 0; j < POSES.length; j++) {
                if (i == j) {
                    continue;
                }
                String label = NAMES[i] + " -> " + NAMES[j];
                java.util.List<double[]> route = Superstructure.planRoute(
                    POSES[i][0], POSES[i][1], POSES[j][0], POSES[j][1]);
                assertFalse(route.isEmpty(), label + " has no route through the free region");
                // Every waypoint, and every straight leg between them, is clear
                for (double[] wp : route) {
                    assertTrue(Superstructure.poseClear(wp[0], wp[1]),
                        String.format("%s waypoint %.1f in / %.1f deg is blocked", label, wp[0], wp[1]));
                }
                for (int k = 0; k < route.size() - 1; k++) {
                    double[] from = route.get(k);
                    double[] to = route.get(k + 1);
                    int steps = 40;
                    for (int t = 0; t <= steps; t++) {
                        double f = (double) t / steps;
                        double h = from[0] + (to[0] - from[0]) * f;
                        double a = from[1] + (to[1] - from[1]) * f;
                        assertTrue(Superstructure.poseClear(h, a),
                            String.format("%s leg %d passes through %.1f in / %.1f deg", label, k, h, a));
                    }
                }
                double[] last = route.get(route.size() - 1);
                assertEquals(Math.max(POSES[j][0], BASE), last[0], 1e-6, label + " must end at the target height");
                assertEquals(POSES[j][1], last[1], 1e-6, label + " must end at the target angle");
                assertTrue(route.size() <= 60, label + " has an implausible number of legs: " + route.size());
            }
        }
    }

    @Test
    void routesFromPosesOnlyManualControlCanCreateAreHandled() {
        // A tucked arm parked high and a high-angle arm at the base are
        // outside the corridors: either a route exists from the snapped pose
        // or the planner falls back to swinging the arm to safe travel, which
        // the rehearsal covers.
        double[][] bad = {{30.0, 0.0}, {2.0, 160.0}, {45.0, 60.0}};
        for (double[] start : bad) {
            MoveTrace trace = Superstructure.rehearsePlan(start[0], start[1], BASE,
                PivotPresetAngles.BASE.getAngle());
            assertTrue(trace.reached, String.format("recovery from %.1f in / %.1f deg deadlocked at %.1f / %.1f",
                start[0], start[1], trace.endHeight, trace.endAngle));
        }
    }

    @Test
    void followingEveryRouteArrivesWithoutStandingStill() {
        // A kinematic run of the real follower: the route, the lookahead that
        // advances waypoints, and the corridor clamp, with each mechanism
        // held to its profile limits. What this pins is the user-visible
        // property - the pair keeps moving instead of stepping through
        // stations - plus arrival and clearance the whole way.
        for (int i = 0; i < POSES.length; i++) {
            for (int j = 0; j < POSES.length; j++) {
                if (i == j) {
                    continue;
                }
                String label = NAMES[i] + " -> " + NAMES[j];
                java.util.List<double[]> route = Superstructure.planRoute(
                    POSES[i][0], POSES[i][1], POSES[j][0], POSES[j][1]);
                assertFalse(route.isEmpty(), label + " has no route");

                double h = POSES[i][0];
                double a = POSES[i][1];
                double vh = 0.0;
                double va = 0.0;
                int index = 0;
                int bothStoppedLoops = 0;
                int loops = 0;
                boolean arrived = false;
                // 5 ms steps: the real controllers run at 1 kHz, and a 20 ms
                // Euler step overshoots a commanded target by up to an inch
                // all by itself, which is an artefact, not the mechanism.
                final double dt = 0.005;
                final double vMaxH = ElevatorConstants.ELEVATOR_MAX_VELOCITY;
                final double aMaxH = ElevatorConstants.ELEVATOR_MAX_ACCELERATION;
                final double vMaxA = frc.robot.Constants.CorAlConstants.CORAL_PIVOT_MAX_VELOCITY;
                final double aMaxA = frc.robot.Constants.CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION;
                for (; loops < 2000 && !arrived; loops++) {
                    while (index < route.size() - 1) {
                        double[] wp = route.get(index);
                        if (Math.abs(h - wp[0]) <= SuperstructureConstants.ROUTE_LOOKAHEAD_IN
                                && Math.abs(a - wp[1]) <= SuperstructureConstants.ROUTE_LOOKAHEAD_DEG) {
                            index++;
                        } else {
                            break;
                        }
                    }
                    double[] wp = route.get(index);
                    double[] cmd = Superstructure.coordinatedCommand(h, a, vh, va, wp[0], wp[1]);
                    vh = approach(h, cmd[0], vh, vMaxH, aMaxH, dt);
                    va = approach(a, cmd[1], va, vMaxA, aMaxA, dt);
                    h += vh * dt;
                    a += va * dt;
                    // How far outside its corridor band the pose ever gets.
                    // Asserting on the DEPTH rather than on a boolean keeps
                    // this meaningful: the table's rows are 5 deg wide and
                    // carry about an inch of model margin, so a hundredth of
                    // an inch past a row boundary is numerical, not physical
                    // - while the excursions this catches (a third of an inch
                    // and up) are real ordering mistakes. Near the start or
                    // goal the row granularity rules, so those are exempt.
                    boolean nearEndpoint = near(h, a, POSES[i]) || near(h, a, POSES[j]);
                    double excess = corridorExcess(h, a);
                    assertTrue(excess <= 0.1 || nearEndpoint,
                        String.format("%s passed %.2f in outside the corridor at %.2f in / %.2f deg",
                            label, excess, h, a));
                    boolean stopped = Math.abs(vh) < 0.5 && Math.abs(va) < 3.0;
                    arrived = index >= route.size() - 1
                        && Math.abs(h - Math.max(POSES[j][0], BASE)) <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR
                        && Math.abs(a - POSES[j][1]) <= frc.robot.Constants.CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
                    if (stopped && !arrived && loops > 8) {
                        bothStoppedLoops++;
                    }
                }
                assertTrue(arrived, String.format("%s did not arrive in %.1f s (stopped at %.1f in / %.1f deg)",
                    label, loops * dt, h, a));
                assertTrue(bothStoppedLoops <= 40,
                    String.format("%s stood still for %.2f s mid-move (step-by-step motion)",
                        label, bothStoppedLoops * dt));
            }
        }
    }

    /**
     * Within one corridor row of the given pose. The table's rows are 5 deg
     * wide and each row carries the worst case over its whole span, with
     * about an inch of model margin on top, so within a row of a validated
     * preset the row granularity is what decides clearance, not the
     * geometry. The presets themselves are checked exactly, separately.
     */
    private static boolean near(double height, double angle, double[] pose) {
        return Math.abs(height - Math.max(pose[0], BASE)) <= 1.0 && Math.abs(angle - pose[1]) <= 5.0;
    }

    /**
     * How far (inches) the pose sits outside the corridor band it should be
     * in: zero when clear, otherwise the distance to the nearest band edge
     * of its angle row.
     */
    private static double corridorExcess(double height, double angle) {
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            if (angle < row[0]) {
                double best = Double.MAX_VALUE;
                best = Math.min(best, bandExcess(height, row[1], row[2]));
                if (!Double.isNaN(row[3])) {
                    best = Math.min(best, bandExcess(height, row[3], row[4]));
                }
                return best;
            }
        }
        return Double.MAX_VALUE;
    }

    private static double bandExcess(double height, double low, double high) {
        if (height < low) {
            return low - height;
        }
        if (height > high) {
            return height - high;
        }
        return 0.0;
    }

    /** One profile step toward a target: trapezoidal, limited by velocity and acceleration. */
    private static double approach(double position, double target, double velocity,
            double maxVelocity, double maxAccel, double dt) {
        double error = target - position;
        double cruise = Math.signum(error) * Math.min(maxVelocity, Math.sqrt(2 * maxAccel * Math.abs(error)));
        double delta = cruise - velocity;
        return velocity + Math.max(-maxAccel * dt, Math.min(maxAccel * dt, delta));
    }

    @Test
    void clampNeverCommandsOutsideTheCorridors() {
        // Sweep the reachable state space: whatever the goals are, the
        // commanded pose must be clear at the measured state it was computed
        // from (this is the safety invariant the streaming command relies on).
        double[][] goals = {{BASE, 0.0}, {52.5, 20.0}, {30.5, 25.0}, {20.5, 160.0}, {12.0, 12.5}};
        for (double h = BASE; h <= ElevatorConstants.ELEVATOR_MAX_POSITION; h += 1.0) {
            for (double a = 0.0; a <= 160.0; a += 2.5) {
                if (!Superstructure.poseClear(h, a)) {
                    continue;
                }
                for (double[] goal : goals) {
                    double[] cmd = Superstructure.coordinatedCommand(h, a, goal[0], goal[1]);
                    assertTrue(Superstructure.poseClear(cmd[0], a),
                        String.format("commanded height %.2f is blocked at the measured angle %.1f", cmd[0], a));
                    assertTrue(Superstructure.poseClear(h, cmd[1]),
                        String.format("commanded angle %.2f is blocked at the measured height %.1f", cmd[1], h));
                    assertTrue(Superstructure.elevatorPathClear(h, cmd[0], a),
                        String.format("path to commanded height %.2f is blocked at angle %.1f", cmd[0], a));
                    assertTrue(Superstructure.pivotPathClear(a, cmd[1], h),
                        String.format("path to commanded angle %.2f is blocked at height %.1f", cmd[1], h));
                }
            }
        }
    }
}
