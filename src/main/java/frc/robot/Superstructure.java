package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.CorAlConstants;
import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.CorAl;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Elastic;

/**
 * Coordinated elevator + CorAl motion ("superstructure") command factory.
 *
 * The elevator and the CorAl claw can collide with the elevator structure
 * at specific combinations of height and angle. The free region of
 * (height, angle) was computed from the robot CAD - see
 * {@link SuperstructureConstants} for the map and how it was validated
 * against the contacts measured on the robot. Every position command in
 * the robot goes through this class, which moves through that region with
 * both mechanisms COORDINATED: every loop each one is commanded toward its
 * own goal, clamped into the window the other one's MEASURED position
 * allows. As one moves, the other's window opens, so the pair flows through
 * the corridors in one continuous motion instead of stopping at waypoints
 * and handing over. The commanded pose is inside the corridors on every
 * loop, so this is safe at any elevator or pivot speed.
 *
 * The shape of the free region, and what it forces:
 *   - Tucked (below ~8 deg) only near the base, so a tuck waits for the
 *     carriage to come down - which it does while the arm is already
 *     rotating toward it.
 *   - Climbing out of the low box needs the arm past ~75 deg, and from
 *     RAISE (100 deg) the carriage may go anywhere; the carriage therefore
 *     climbs as fast as the arm's sweep unlocks height, with no dwell.
 *   - The top of travel is clear only at 22.5 deg or less, and the high
 *     scoring pose cannot be rotated into up there - so on the way to L4
 *     the carriage rises behind the arm as it comes down, each unlocking
 *     the other, and on the way back the arm swings up as the carriage
 *     descends.
 *   - A few poses cannot be reached by clamped motion alone (the arm needs
 *     the carriage lower and the carriage needs the arm higher). Those get
 *     one extra phase first: the arm's goal becomes RAISE while the
 *     carriage keeps working toward the target height, handing over the
 *     moment the arm gets there. The planner decides this by rehearsing
 *     the move as pure geometry ({@link #rehearse}) before it starts.
 *
 * NOTE: the operator's manual stick controls bypass these interlocks - they
 * command the subsystems directly and rely on the operator watching the
 * mechanism.
 */
public class Superstructure {

    /**
     * The pose family most recently commanded. Consumed by the vision
     * tracker to pick the right AprilTag alignment goal (e.g. reef tags are
     * approached flush for L2-L4 but held at a standoff for L1).
     */
    public enum Goal {
        STOW,
        CORAL_L1,
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        ALGAE_LOW,
        ALGAE_HIGH,
        ALGAE_SCORE
    }

    /** Safe travel angle in degrees (RAISE preset): clear at every height. */
    private static final double SAFE_ANGLE = PivotPresetAngles.RAISE.getAngle();
    private static final double TOL = SuperstructureConstants.SAFE_ANGLE_TOLERANCE;

    private final Elevator elevator;
    private final CorAl coral;

    /** Last commanded pose family (drives vision alignment goals). */
    private Goal currentGoal = Goal.STOW;

    public Superstructure(Elevator elevator, CorAl coral) {
        this.elevator = elevator;
        this.coral = coral;
    }

    /** The most recently commanded pose family. */
    public Goal getGoal() {
        return currentGoal;
    }

    /** Records the commanded pose family (runs even if the move is interrupted). */
    private Command setGoal(Goal goal) {
        return Commands.runOnce(() -> currentGoal = goal);
    }

    // ==================================================================
    // CAD free-region predicates (pure functions of height and angle)
    // ==================================================================

    /**
     * True if the pose lies inside the CAD free corridors
     * ({@link SuperstructureConstants#FREE_CORRIDORS}): the height is in
     * either corridor of the angle's row. Angles beyond the table are not
     * clear.
     */
    public static boolean poseClear(double height, double angle) {
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            if (angle < row[0]) {
                boolean low = height >= row[1] && height <= row[2];
                boolean high = !Double.isNaN(row[3]) && height >= row[3] && height <= row[4];
                return low || high;
            }
        }
        return false;
    }

    /** True if every height from {@code from} to {@code to} (0.25 in steps) is clear at {@code angle}. */
    public static boolean elevatorPathClear(double from, double to, double angle) {
        double lo = Math.min(from, to), hi = Math.max(from, to);
        for (double h = lo; h <= hi + 1e-9; h += 0.25) {
            if (!poseClear(Math.min(h, hi), angle)) {
                return false;
            }
        }
        return poseClear(hi, angle);
    }

    /** True if every angle from {@code from} to {@code to} (0.5 deg steps) is clear at {@code height}. */
    public static boolean pivotPathClear(double from, double to, double height) {
        double lo = Math.min(from, to), hi = Math.max(from, to);
        for (double a = lo; a <= hi + 1e-9; a += 0.5) {
            if (!poseClear(height, Math.min(a, hi))) {
                return false;
            }
        }
        return poseClear(height, hi);
    }

    /**
     * Audits the operator presets against the corridors at startup. Returns
     * an empty string when every preset is clear, otherwise a message naming
     * the ones the CAD model puts inside a contact band (the Dashboard shows
     * it as an info alert). Presets are still commanded as given - the
     * operator owns them - this only makes a known-tight pose visible.
     */
    public static String presetAuditMessage() {
        List<String> tight = new ArrayList<>();
        double[][] presets = {
            {ElevatorConstants.ELEVATOR_ZERO_HEIGHT, PivotPresetAngles.BASE.getAngle()},
            {PresetHeights.CORAL_L1.getHeight(), PivotPresetAngles.CORAL_L1.getAngle()},
            {PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle()},
            {PresetHeights.CORAL_L3.getHeight(), PivotPresetAngles.CORAL_L3.getAngle()},
            {PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle()},
            {PresetHeights.ALGAE_LOW_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()},
            {PresetHeights.ALGAE_HIGH_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()},
            {PresetHeights.ALGAE_SCORE.getHeight(), PivotPresetAngles.ALGAE_SCORE.getAngle()},
        };
        String[] names = {"BASE", "L1", "L2", "L3", "L4", "ALGAE LOW", "ALGAE HIGH", "ALGAE SCORE"};
        for (int i = 0; i < presets.length; i++) {
            double h = Math.max(presets[i][0], ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
            if (!poseClear(h, presets[i][1])) {
                tight.add(String.format("%s (%.1f in, %.1f deg)", names[i], h, presets[i][1]));
            }
        }
        return tight.isEmpty() ? ""
            : "CAD model puts these presets inside a contact band (< 1 in): " + String.join(", ", tight)
                + ". They are commanded as set; watch them.";
    }

    /** One-line description of how the planner moves, for the dashboard. */
    public static String motionSummary() {
        return String.format(
            "coordinated: both mechanisms stream targets every loop, each clamped to the CAD corridor window the "
                + "other's measured position allows (arm sampled every %.1f deg, carriage every %.1f in, margins "
                + "%.2f in / %.1f deg); a move that would deadlock gets one phase at RAISE %.0f deg first",
            SuperstructureConstants.COORD_ANGLE_STEP, SuperstructureConstants.COORD_HEIGHT_STEP,
            SuperstructureConstants.COORD_HEIGHT_MARGIN, SuperstructureConstants.COORD_ANGLE_MARGIN,
            SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE);
    }

    // ==================================================================
    // Coordinated motion core
    // ==================================================================
    // Both mechanisms are commanded EVERY loop, each toward its own goal but
    // CLAMPED into the window the other one's MEASURED position allows. The
    // windows come straight from the CAD corridors:
    //
    //   elevator window = the corridor band containing the carriage now,
    //       intersected over every angle the arm will sweep between where it
    //       is and where it is going - so the carriage never commits to a
    //       height the arm is about to block;
    //   arm window     = the clear angle range containing the arm now,
    //       intersected over every height the carriage will pass through.
    //
    // As one mechanism moves, the other's window opens, so the pair flows
    // through the corridors continuously instead of stopping at waypoints.
    // The commanded pose is inside the corridors on every single loop, which
    // is a stronger guarantee than the old staged gates gave, and it holds at
    // any speed because it is recomputed from measured state.

    /** Corridor band (floor, ceiling) containing {@code height} at {@code angle}; null if that pose is blocked. */
    private static final double BAND_EDGE_EPSILON = 0.01;

    private static double[] heightBand(double angle, double height) {
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            if (angle < row[0]) {
                if (height >= row[1] && height <= row[2]) {
                    return new double[] {row[1], row[2]};
                }
                if (!Double.isNaN(row[3]) && height >= row[3] && height <= row[4]) {
                    return new double[] {row[3], row[4]};
                }
                return null;
            }
        }
        return null;
    }

    /**
     * Contiguous clear angle range containing {@code angle} at
     * {@code height}, read exactly off the corridor rows (each row covers
     * the angles up to its first element), or null if that pose is blocked.
     */
    private static double[] angleBand(double height, double angleIn) {
        double angle = clamp(angleIn, CorAlConstants.CORAL_PIVOT_MIN_ANGLE, CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        double[][] rows = SuperstructureConstants.FREE_CORRIDORS;
        int index = -1;
        double[] lows = new double[rows.length];
        boolean[] clear = new boolean[rows.length];
        double rowLo = CorAlConstants.CORAL_PIVOT_MIN_ANGLE;
        for (int i = 0; i < rows.length; i++) {
            double[] row = rows[i];
            lows[i] = rowLo;
            clear[i] = (height >= row[1] && height <= row[2])
                || (!Double.isNaN(row[3]) && height >= row[3] && height <= row[4]);
            if (angle >= rowLo && angle < row[0]) {
                index = i;
            }
            rowLo = row[0];
        }
        if (index < 0 || !clear[index]) {
            return null;
        }
        int first = index;
        while (first > 0 && clear[first - 1]) {
            first--;
        }
        int last = index;
        while (last < rows.length - 1 && clear[last + 1]) {
            last++;
        }
        // A row's first element is the angle it stops covering, so the top of
        // a run that ends at a blocked row is just BELOW that bound.
        double top = last == rows.length - 1 ? rows[last][0] : rows[last][0] - BAND_EDGE_EPSILON;
        return new double[] {
            Math.max(lows[first], CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
            Math.min(top, CorAlConstants.CORAL_PIVOT_MAX_ANGLE),
        };
    }

    /** How far a mechanism travels if it starts braking now (signed). */
    private static double brakingDistance(double velocity, double maxAcceleration) {
        return Math.signum(velocity) * velocity * velocity / (2 * maxAcceleration);
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    /**
     * Shrinks a window by a margin, so a mechanism that overshoots its
     * command slightly still lands clear. An edge that is the mechanism's
     * own travel limit is NOT shrunk (a hard stop is not a contact band),
     * and neither the current position nor a goal that is itself clear is
     * ever excluded - otherwise the clamp would fight the pose it is trying
     * to reach.
     */
    private static double[] withMargin(double lo, double hi, double margin,
            double goal, double travelMin, double travelMax, double velocity, double maxAcceleration) {
        double l = lo <= travelMin + 1e-9 ? travelMin : lo + margin;
        double h = hi >= travelMax - 1e-9 ? travelMax : hi - margin;
        // Leave room to stop: the edge the mechanism is heading for has to
        // be far enough away that braking from the current speed fits inside
        // it, or the command would be a point the mechanism coasts past.
        double braking = Math.abs(brakingDistance(velocity, maxAcceleration));
        if (velocity > 0 && hi < travelMax - 1e-9) {
            h -= braking;
        } else if (velocity < 0 && lo > travelMin + 1e-9) {
            l += braking;
        }
        // A goal that is INSIDE the window keeps its place there: the margin
        // exists to absorb overshoot, not to stop short of a validated pose.
        if (goal >= lo - 1e-9 && goal <= hi + 1e-9) {
            l = Math.min(l, goal);
            h = Math.max(h, goal);
        }
        // Deliberately NOT widened to include the current position: a
        // mechanism that has drifted past the margin line is commanded back
        // to it. Widening would ratchet - each loop's creep would raise the
        // limit that was supposed to stop the creep.
        return new double[] {Math.min(l, h), Math.max(l, h)};
    }

    /**
     * The clamped {height, angle} pair to command right now, given the
     * measured pose and where each mechanism is trying to go. Pure geometry,
     * so the planner can rehearse it and the tests can pin it.
     */
    public static double[] coordinatedCommand(double height, double angle, double heightGoal, double angleGoal) {
        return coordinatedCommand(height, angle, 0.0, 0.0, heightGoal, angleGoal);
    }

    /**
     * The clamped {height, angle} pair to command right now, given the
     * measured pose, the measured VELOCITIES and where each mechanism is
     * trying to go.
     *
     * The velocities matter: a mechanism cannot stop at its command, only
     * decelerate into it, so each one's window is computed over everything
     * the OTHER one can still reach before it could brake to a stop
     * (v^2/2a). Without that the arm at full speed, which needs about 65
     * degrees to stop, would have its window closed by a carriage that had
     * every right to move - and would coast straight through the edge of a
     * corridor.
     */
    public static double[] coordinatedCommand(double heightIn, double angleIn, double heightVelocity,
            double angleVelocity, double heightGoal, double angleGoal) {
        double height = clamp(heightIn, ElevatorConstants.ELEVATOR_ZERO_HEIGHT,
            ElevatorConstants.ELEVATOR_MAX_POSITION);
        double angle = clamp(angleIn, CorAlConstants.CORAL_PIVOT_MIN_ANGLE, CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        double target = Math.max(heightGoal, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        // A pose outside the map (only reachable by manual control) recovers
        // by holding the carriage and swinging the arm to the safe travel
        // angle, which is clear at every height.
        if (!poseClear(height, angle)) {
            return new double[] {height, SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE};
        }

        // 1. How far the arm may rotate toward its goal at the height we are at.
        double[] aBand = angleBand(height, angle);
        if (aBand == null) {
            return new double[] {height, SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE};
        }
        double[] reachWindow = withMargin(aBand[0], aBand[1], SuperstructureConstants.COORD_ANGLE_MARGIN,
            angleGoal, CorAlConstants.CORAL_PIVOT_MIN_ANGLE, CorAlConstants.CORAL_PIVOT_MAX_ANGLE,
            angleVelocity, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        double armReach = clamp(angleGoal, reachWindow[0], reachWindow[1]);

        // 2. Heights that stay clear for every angle the arm can still be at:
        //    anywhere between here, where it is aiming, and where it would
        //    end up if it started braking this instant.
        double heightLo = ElevatorConstants.ELEVATOR_ZERO_HEIGHT;
        double heightHi = ElevatorConstants.ELEVATOR_MAX_POSITION;
        double armBraking = angle + brakingDistance(angleVelocity, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        double from = Math.min(Math.min(angle, armReach), armBraking);
        double to = Math.max(Math.max(angle, armReach), armBraking);
        for (double alpha = from; alpha < to + SuperstructureConstants.COORD_ANGLE_STEP;
                alpha += SuperstructureConstants.COORD_ANGLE_STEP) {
            double[] band = heightBand(Math.min(alpha, to), height);
            if (band == null) {
                break;
            }
            heightLo = Math.max(heightLo, band[0]);
            heightHi = Math.min(heightHi, band[1]);
        }
        double[] hWindow = withMargin(heightLo, heightHi, SuperstructureConstants.COORD_HEIGHT_MARGIN,
            target, ElevatorConstants.ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_MAX_POSITION,
            heightVelocity, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        double heightCommand = clamp(target, hWindow[0], hWindow[1]);

        // 3. Mirror: angles that stay clear for every height the carriage will pass.
        double angleLo = CorAlConstants.CORAL_PIVOT_MIN_ANGLE;
        double angleHi = CorAlConstants.CORAL_PIVOT_MAX_ANGLE;
        double carriageBraking = height + brakingDistance(heightVelocity, ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
        double hFrom = Math.min(Math.min(height, heightCommand), carriageBraking);
        double hTo = Math.max(Math.max(height, heightCommand), carriageBraking);
        for (double h = hFrom; h < hTo + SuperstructureConstants.COORD_HEIGHT_STEP;
                h += SuperstructureConstants.COORD_HEIGHT_STEP) {
            double[] band = angleBand(Math.min(h, hTo), angle);
            if (band == null) {
                break;
            }
            angleLo = Math.max(angleLo, band[0]);
            angleHi = Math.min(angleHi, band[1]);
        }
        double[] aWindow = withMargin(angleLo, angleHi, SuperstructureConstants.COORD_ANGLE_MARGIN,
            angleGoal, CorAlConstants.CORAL_PIVOT_MIN_ANGLE, CorAlConstants.CORAL_PIVOT_MAX_ANGLE,
            angleVelocity, CorAlConstants.CORAL_PIVOT_MAX_ACCELERATION);
        double angleCommand = clamp(armReach, aWindow[0], aWindow[1]);

        return new double[] {heightCommand, angleCommand};
    }

    // ==================================================================
    // Route planning through the free region
    // ==================================================================
    // A move is planned as a ROUTE: a short list of {height, angle}
    // waypoints through the CAD free region, found by a shortest-time search
    // over a grid of the corridor map and then straightened by dropping any
    // waypoint the route can see past. Diagonal steps cost the same as the
    // longer of their two axes, so the search prefers moving both mechanisms
    // at once, and the straightening turns the result into a couple of long
    // diagonal legs. The follower aims at the next waypoint with a lookahead,
    // so it retargets before the profiles decelerate: the mechanisms flow
    // along the route instead of stopping at each leg.
    //
    // This replaces the old hand-written staging (station heights, rotate
    // windows, staged exits). It cannot walk into a corner the way clamping
    // alone can, because it searches the whole reachable region: if a route
    // exists on the grid it is found, and if none does the caller falls back
    // to swinging the arm to the safe travel angle first.

    private static final double BOX_HEIGHT_STEP = 0.5;
    private static final double BOX_ANGLE_STEP = 2.5;
    private static final double ENDPOINT_RELIEF_IN = 1.0;
    private static final double ENDPOINT_RELIEF_DEG = 2.5;
    private static final int MAX_LEG_SKIP = 10;
    private static final double GRID_HEIGHT_STEP = 0.5;
    private static final double GRID_ANGLE_STEP = 2.5;

    private static boolean[][] freeGrid;
    private static int gridHeights;
    private static int gridAngles;

    private static double gridHeight(int i) {
        return ElevatorConstants.ELEVATOR_ZERO_HEIGHT + i * GRID_HEIGHT_STEP;
    }

    private static double gridAngle(int j) {
        return CorAlConstants.CORAL_PIVOT_MIN_ANGLE + j * GRID_ANGLE_STEP;
    }

    /**
     * Builds (once) the grid of poses the route search may use. An interior
     * node has to be clear with a margin on both axes, so a route never hugs
     * the edge of a contact band; the start and goal are added separately by
     * the search, because a validated preset is allowed to sit on an edge.
     */
    private static synchronized void buildGrid() {
        if (freeGrid != null) {
            return;
        }
        gridHeights = (int) Math.floor(
            (ElevatorConstants.ELEVATOR_MAX_POSITION - ElevatorConstants.ELEVATOR_ZERO_HEIGHT) / GRID_HEIGHT_STEP) + 1;
        gridAngles = (int) Math.floor(
            (CorAlConstants.CORAL_PIVOT_MAX_ANGLE - CorAlConstants.CORAL_PIVOT_MIN_ANGLE) / GRID_ANGLE_STEP) + 1;
        freeGrid = new boolean[gridHeights][gridAngles];
        for (int i = 0; i < gridHeights; i++) {
            for (int j = 0; j < gridAngles; j++) {
                freeGrid[i][j] = clearWithMargin(gridHeight(i), gridAngle(j));
            }
        }
    }

    /** True if the pose is clear with the planning margin on both axes. */
    private static boolean clearWithMargin(double height, double angle) {
        double hm = SuperstructureConstants.COORD_HEIGHT_MARGIN;
        double am = SuperstructureConstants.COORD_ANGLE_MARGIN;
        return poseClear(height, angle)
            && poseClear(Math.max(height - hm, ElevatorConstants.ELEVATOR_ZERO_HEIGHT), angle)
            && poseClear(Math.min(height + hm, ElevatorConstants.ELEVATOR_MAX_POSITION), angle)
            && poseClear(height, Math.max(angle - am, CorAlConstants.CORAL_PIVOT_MIN_ANGLE))
            && poseClear(height, Math.min(angle + am, CorAlConstants.CORAL_PIVOT_MAX_ANGLE));
    }

    /**
     * True if the whole RECTANGLE between two poses is inside the corridors.
     *
     * This is the condition a leg of the route has to meet, and it is
     * stronger than "the straight line between them is clear": both
     * mechanisms are commanded to the waypoint at once and each runs its own
     * profile, so the pair can pass through any pose in the box between the
     * two - the carriage ahead of the arm, the arm ahead of the carriage, or
     * anything between. A leg whose box is clear is therefore safe however
     * the two interleave, which is what lets them both just go instead of
     * being clamped and re-clamped against each other.
     *
     * Interior poses must clear the planning margin as well; the two ends
     * are exempt when they are the move's real start and goal, since a
     * validated preset is allowed to sit on a corridor edge (L3 does).
     */
    private static boolean boxClear(double h0, double a0, double h1, double a1,
            boolean reliefAtStart, boolean reliefAtEnd) {
        double hLo = Math.min(h0, h1);
        double hHi = Math.max(h0, h1);
        double aLo = Math.min(a0, a1);
        double aHi = Math.max(a0, a1);
        int hSteps = (int) Math.ceil((hHi - hLo) / BOX_HEIGHT_STEP);
        int aSteps = (int) Math.ceil((aHi - aLo) / BOX_ANGLE_STEP);
        for (int i = 0; i <= hSteps; i++) {
            double h = hSteps == 0 ? hLo : hLo + (hHi - hLo) * i / hSteps;
            for (int j = 0; j <= aSteps; j++) {
                double a = aSteps == 0 ? aLo : aLo + (aHi - aLo) * j / aSteps;
                if (!poseClear(h, a)) {
                    return false;
                }
                boolean atStart = reliefAtStart
                    && Math.abs(h - h0) <= ENDPOINT_RELIEF_IN && Math.abs(a - a0) <= ENDPOINT_RELIEF_DEG;
                boolean atEnd = reliefAtEnd
                    && Math.abs(h - h1) <= ENDPOINT_RELIEF_IN && Math.abs(a - a1) <= ENDPOINT_RELIEF_DEG;
                if (!atStart && !atEnd && !clearWithMargin(h, a)) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Shortest-time route from one pose to another through the free region,
     * as {height, angle} waypoints ending at the goal, or an empty list when
     * the grid has no route. The first waypoint is the start.
     */
    public static List<double[]> planRoute(double startHeight, double startAngle,
            double goalHeight, double goalAngle) {
        buildGrid();
        double sh = clamp(startHeight, ElevatorConstants.ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_MAX_POSITION);
        double sa = clamp(startAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE, CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        double gh = clamp(Math.max(goalHeight, ElevatorConstants.ELEVATOR_ZERO_HEIGHT),
            ElevatorConstants.ELEVATOR_ZERO_HEIGHT, ElevatorConstants.ELEVATOR_MAX_POSITION);
        double ga = clamp(goalAngle, CorAlConstants.CORAL_PIVOT_MIN_ANGLE, CorAlConstants.CORAL_PIVOT_MAX_ANGLE);

        int si = nearestIndex(sh, GRID_HEIGHT_STEP, gridHeights, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        int sj = nearestIndex(sa, GRID_ANGLE_STEP, gridAngles, CorAlConstants.CORAL_PIVOT_MIN_ANGLE);
        int gi = nearestIndex(gh, GRID_HEIGHT_STEP, gridHeights, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        int gj = nearestIndex(ga, GRID_ANGLE_STEP, gridAngles, CorAlConstants.CORAL_PIVOT_MIN_ANGLE);

        // Dijkstra over the grid; cost is the time the slower axis needs, so
        // diagonal steps (both mechanisms moving) are the cheap ones.
        int nodes = gridHeights * gridAngles;
        double[] dist = new double[nodes];
        int[] prev = new int[nodes];
        boolean[] done = new boolean[nodes];
        java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(prev, -1);
        int startNode = si * gridAngles + sj;
        dist[startNode] = 0.0;
        java.util.PriorityQueue<int[]> queue = new java.util.PriorityQueue<>(
            (x, y) -> Double.compare(dist[x[0]], dist[y[0]]));
        queue.add(new int[] {startNode});
        int goalNode = gi * gridAngles + gj;
        while (!queue.isEmpty()) {
            int node = queue.poll()[0];
            if (done[node]) {
                continue;
            }
            done[node] = true;
            if (node == goalNode) {
                break;
            }
            int i = node / gridAngles;
            int j = node % gridAngles;
            for (int di = -1; di <= 1; di++) {
                for (int dj = -1; dj <= 1; dj++) {
                    if (di == 0 && dj == 0) {
                        continue;
                    }
                    int ni = i + di;
                    int nj = j + dj;
                    if (ni < 0 || ni >= gridHeights || nj < 0 || nj >= gridAngles) {
                        continue;
                    }
                    int next = ni * gridAngles + nj;
                    boolean endpoint = next == goalNode || next == startNode;
                    if (!freeGrid[ni][nj] && !endpoint) {
                        continue;
                    }
                    if (!poseClear(gridHeight(ni), gridAngle(nj))) {
                        continue;
                    }
                    double cost = Math.max(Math.abs(di) * GRID_HEIGHT_STEP / ElevatorConstants.ELEVATOR_MAX_VELOCITY,
                        Math.abs(dj) * GRID_ANGLE_STEP / CorAlConstants.CORAL_PIVOT_MAX_VELOCITY);
                    if (dist[node] + cost < dist[next]) {
                        dist[next] = dist[node] + cost;
                        prev[next] = node;
                        queue.add(new int[] {next});
                    }
                }
            }
        }

        List<double[]> route = new ArrayList<>();
        if (Double.isInfinite(dist[goalNode])) {
            return route;
        }
        List<double[]> reversed = new ArrayList<>();
        for (int node = goalNode; node != -1; node = prev[node]) {
            reversed.add(new double[] {gridHeight(node / gridAngles), gridAngle(node % gridAngles)});
            if (node == startNode) {
                break;
            }
        }
        java.util.Collections.reverse(reversed);
        // Real endpoints instead of their grid stand-ins
        reversed.set(0, new double[] {sh, sa});
        reversed.set(reversed.size() - 1, new double[] {gh, ga});

        // Straighten into as few legs as possible: from each waypoint, reach
        // for the furthest one whose BOX is still clear. Bounded so planning
        // stays cheap (it runs when the button is pressed).
        route.add(reversed.get(0));
        int at = 0;
        while (at < reversed.size() - 1) {
            int furthest = at + 1;
            int limit = Math.min(reversed.size() - 1, at + MAX_LEG_SKIP);
            for (int k = limit; k > at; k--) {
                double[] from = reversed.get(at);
                double[] to = reversed.get(k);
                if (boxClear(from[0], from[1], to[0], to[1], at == 0, k == reversed.size() - 1)) {
                    furthest = k;
                    break;
                }
            }
            route.add(reversed.get(furthest));
            at = furthest;
        }
        return route;
    }

    private static int nearestIndex(double value, double step, int count, double origin) {
        return (int) Math.round(clamp((value - origin) / step, 0, count - 1));
    }

    /**
     * The angle the arm should be working toward right now for a move to
     * {@code targetHeight} / {@code targetAngle}.
     *
     * The target angle is only valid inside one corridor band - L4's 20 deg
     * lives in the 37.5-53.5 in band, a tuck lives below 8.5 in - so while
     * the carriage is somewhere else, aiming the arm straight at the target
     * angle would walk the pair into a corner where the carriage cannot
     * cross into the target band and the arm cannot get back out of the way.
     * So until the carriage is IN that band, the arm works toward the safe
     * travel angle, which is clear at every height; once the carriage is in
     * the band, the arm turns for the target. Both mechanisms keep moving
     * throughout: this only changes what the arm is aiming at, and the
     * clamp keeps every commanded pose inside the corridors.
     */
    public static double armGoalFor(double height, double targetHeight, double targetAngle) {
        double target = Math.max(targetHeight, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        double[] band = heightBand(targetAngle, target);
        if (band != null
                && height >= band[0] - SuperstructureConstants.COORD_HEIGHT_MARGIN
                && height <= band[1] + SuperstructureConstants.COORD_HEIGHT_MARGIN) {
            return targetAngle;
        }
        return SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE;
    }

    /** Result of rehearsing a coordinated move as pure geometry. */
    public static final class MoveTrace {
        public final boolean reached;
        public final boolean everBlocked;
        public final int steps;
        public final double endHeight;
        public final double endAngle;

        MoveTrace(boolean reached, boolean everBlocked, int steps, double endHeight, double endAngle) {
            this.reached = reached;
            this.everBlocked = everBlocked;
            this.steps = steps;
            this.endHeight = endHeight;
            this.endAngle = endAngle;
        }
    }

    /**
     * Rehearses the clamped controller from a starting pose, stepping each
     * mechanism toward its clamped command in the ratio of their speeds. No
     * hardware and no dynamics: this answers "does this move flow to the
     * goal, or does it deadlock and need the safe-travel detour", and the
     * tests use it to check that every pose it passes through is clear.
     */
    public static MoveTrace rehearse(double startHeight, double startAngle, double heightGoal, double angleGoal) {
        return rehearse(startHeight, startAngle, heightGoal, angleGoal, false);
    }

    private static MoveTrace rehearse(double startHeight, double startAngle, double heightGoal, double angleGoal,
            boolean untilArmSafe) {
        double h = startHeight;
        double a = startAngle;
        boolean everBlocked = !poseClear(h, a);
        for (int i = 0; i < SuperstructureConstants.PLAN_MAX_STEPS; i++) {
            boolean done = untilArmSafe
                ? a >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE - TOL
                : atPose(h, a, heightGoal, angleGoal);
            if (done) {
                return new MoveTrace(true, everBlocked, i, h, a);
            }
            double armGoal = untilArmSafe ? SAFE_ANGLE : armGoalFor(h, heightGoal, angleGoal);
            double[] cmd = coordinatedCommand(h, a, heightGoal, armGoal);
            double dh = clamp(cmd[0] - h, -SuperstructureConstants.PLAN_HEIGHT_STEP, SuperstructureConstants.PLAN_HEIGHT_STEP);
            double da = clamp(cmd[1] - a, -SuperstructureConstants.PLAN_ANGLE_STEP, SuperstructureConstants.PLAN_ANGLE_STEP);
            if (Math.abs(dh) < 1e-6 && Math.abs(da) < 1e-6) {
                return new MoveTrace(false, everBlocked, i, h, a); // deadlocked
            }
            h += dh;
            a += da;
            if (!poseClear(h, a)) {
                everBlocked = true;
            }
        }
        boolean reached = untilArmSafe
            ? a >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE - TOL
            : atPose(h, a, heightGoal, angleGoal);
        return new MoveTrace(reached, everBlocked, SuperstructureConstants.PLAN_MAX_STEPS, h, a);
    }

    /**
     * Rehearses the WHOLE plan the robot would run for this move, detour
     * included, exactly as {@link #planMove} chooses it. The tests use this
     * to check that every pose a preset-to-preset move passes through is
     * inside the corridors and that it always arrives.
     */
    public static MoveTrace rehearsePlan(double startHeight, double startAngle, double heightGoal, double angleGoal) {
        MoveTrace direct = rehearse(startHeight, startAngle, heightGoal, angleGoal);
        if (direct.reached) {
            return direct;
        }
        // Backstop for a pose only manual control can create: get the arm to
        // the safe travel angle first, then run the move.
        MoveTrace toSafe = rehearse(startHeight, startAngle, heightGoal, SAFE_ANGLE, true);
        MoveTrace rest = rehearse(toSafe.endHeight, toSafe.endAngle, heightGoal, angleGoal);
        return new MoveTrace(rest.reached, toSafe.everBlocked || rest.everBlocked,
            toSafe.steps + rest.steps, rest.endHeight, rest.endAngle);
    }

    private static boolean atPose(double height, double angle, double heightGoal, double angleGoal) {
        return Math.abs(height - Math.max(heightGoal, ElevatorConstants.ELEVATOR_ZERO_HEIGHT)) <= ElevatorConstants.ELEVATOR_ALLOWED_ERROR
            && Math.abs(angle - angleGoal) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }

    // ==================================================================
    // Measured-state gates and mechanism steps
    // ==================================================================

    private boolean armAtLeast(double angle) {
        return coral.getPivotAngle() >= angle - TOL;
    }

    private Command armTo(double angle) {
        return Commands.runOnce(() -> coral.setPivotAngle(angle), coral);
    }

    private Command elevatorTo(double height) {
        return Commands.runOnce(() -> elevator.setPosition(height), elevator);
    }

    /** True when both mechanisms have settled at their commanded targets. */
    private boolean atTargets() {
        return elevator.isAtTargetPosition() && coral.isAtTargetAngle();
    }

    /** True when both mechanisms have stopped moving, wherever they ended up. */
    private boolean bothStopped() {
        return Math.abs(elevator.getVelocity()) <= SuperstructureConstants.SETTLE_STOPPED_ELEVATOR_IN_S
            && Math.abs(coral.getPivotVelocity()) <= SuperstructureConstants.SETTLE_STOPPED_PIVOT_DEG_S;
    }

    /**
     * Final settle wait for a planned move. It ends when both mechanisms are
     * inside their at-target tolerances OR both have stopped moving,
     * whichever comes first, with a timeout as a backstop so a mechanism
     * that stalls cannot deadlock the command (and with it the operator's
     * default manual controls). Ending on "stopped" is what lets the
     * tolerances be tight: once motion has ceased the closed loops are
     * holding their latched setpoints and waiting longer cannot improve the
     * pose. A short minimum dwell keeps "stopped" from firing before the
     * mechanisms have started.
     */
    private Command settle() {
        return Commands.waitSeconds(SuperstructureConstants.SETTLE_MIN_SECONDS)
            .andThen(Commands.waitUntil(() -> atTargets() || bothStopped()))
            .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS);
    }

    // ------------------------------------------------------------------
    // The streaming command
    // ------------------------------------------------------------------

    /** Last values sent to the controllers, so identical setpoints are not re-sent. */
    private double lastHeightCommand = Double.NaN;
    private double lastAngleCommand = Double.NaN;

    /**
     * Streams clamped targets to both mechanisms until {@code done} is true.
     * The goals are suppliers so a phase can follow a moving goal, and the
     * clamp is recomputed from measured state every loop.
     */
    private Command stream(DoubleSupplier heightGoal, DoubleSupplier angleGoal, BooleanSupplier done) {
        return Commands.run(() -> {
            double h = elevator.getCurrentPosition();
            double a = coral.getPivotAngle();
            double[] cmd = coordinatedCommand(h, a, elevator.getVelocity(), coral.getPivotVelocity(),
                heightGoal.getAsDouble(), angleGoal.getAsDouble());
            if (Double.isNaN(lastHeightCommand) || Math.abs(cmd[0] - lastHeightCommand) > SuperstructureConstants.COORD_SEND_EPSILON_IN) {
                elevator.setPosition(cmd[0]);
                lastHeightCommand = cmd[0];
            }
            if (Double.isNaN(lastAngleCommand) || Math.abs(cmd[1] - lastAngleCommand) > SuperstructureConstants.COORD_SEND_EPSILON_DEG) {
                coral.setPivotAngle(cmd[1]);
                lastAngleCommand = cmd[1];
            }
        }, elevator, coral)
            .beforeStarting(() -> {
                lastHeightCommand = Double.NaN;
                lastAngleCommand = Double.NaN;
            })
            .until(done)
            .withTimeout(SuperstructureConstants.COORD_TIMEOUT_SECONDS);
    }

    // ==================================================================
    // Motion planner
    // ==================================================================

    /**
     * Returns a command that moves to the given pose safely and as fast as
     * the free corridors allow. Deferred so the plan is built from the REAL
     * mechanism state at the moment the command starts.
     */
    private Command moveTo(PresetHeights height, PivotPresetAngles angle) {
        return Commands.defer(() -> planMove(height.getHeight(), angle.getAngle()),
            Set.of(elevator, coral));
    }

    /**
     * Builds the move. Most poses are one continuous coordinated phase. When
     * the corridors make the direct move deadlock - the classic case is a
     * scoring pose at height, where the carriage cannot come down until the
     * arm swings up and the arm cannot swing up until the carriage comes
     * down - the arm is first given the safe travel angle as its goal while
     * the carriage keeps working toward the target height. That phase hands
     * over the moment the arm reaches the safe angle, so the mechanisms
     * never stop; only the arm's goal changes underneath them.
     */
    /** Index of the waypoint being aimed at, for the follower. */
    private int routeIndex = 0;

    /**
     * Follows a planned route: aims both mechanisms at the next waypoint,
     * advancing to the following one as soon as the pose is within the
     * lookahead, so the profiles retarget before they decelerate and the
     * motion runs through the legs in one flow. Every commanded pair still
     * goes through the corridor clamp, which can only restrict it.
     */
    private Command follow(List<double[]> route) {
        return Commands.run(() -> {
            double h = elevator.getCurrentPosition();
            double a = coral.getPivotAngle();
            while (routeIndex < route.size() - 1) {
                double[] wp = route.get(routeIndex);
                boolean close = Math.abs(h - wp[0]) <= SuperstructureConstants.ROUTE_LOOKAHEAD_IN
                    && Math.abs(a - wp[1]) <= SuperstructureConstants.ROUTE_LOOKAHEAD_DEG;
                if (!close) {
                    break;
                }
                // Advance early - that is what keeps the legs flowing into
                // one motion - but only while the NEXT leg's box is clear
                // from where the mechanisms actually are. The route's boxes
                // are drawn from waypoint to waypoint; skipping ahead while
                // still short of one would leave that guarantee behind.
                double[] next = route.get(routeIndex + 1);
                if (!boxClear(h, a, next[0], next[1], true, routeIndex + 1 == route.size() - 1)) {
                    break;
                }
                routeIndex++;
            }
            double[] wp = route.get(routeIndex);
            // The leg's box is clear, so both mechanisms can simply go for
            // the waypoint: whatever order they arrive in, every pose in
            // between is inside the corridors. The clamp is kept only as a
            // veto for a pose the route could not have produced (manual
            // control between plan and execution).
            double[] cmd = poseClear(wp[0], wp[1])
                ? new double[] {Math.max(wp[0], ElevatorConstants.ELEVATOR_ZERO_HEIGHT), wp[1]}
                : coordinatedCommand(h, a, elevator.getVelocity(), coral.getPivotVelocity(), wp[0], wp[1]);
            if (Double.isNaN(lastHeightCommand)
                    || Math.abs(cmd[0] - lastHeightCommand) > SuperstructureConstants.COORD_SEND_EPSILON_IN) {
                elevator.setPosition(cmd[0]);
                lastHeightCommand = cmd[0];
            }
            if (Double.isNaN(lastAngleCommand)
                    || Math.abs(cmd[1] - lastAngleCommand) > SuperstructureConstants.COORD_SEND_EPSILON_DEG) {
                coral.setPivotAngle(cmd[1]);
                lastAngleCommand = cmd[1];
            }
        }, elevator, coral)
            .beforeStarting(() -> {
                routeIndex = 0;
                lastHeightCommand = Double.NaN;
                lastAngleCommand = Double.NaN;
            })
            .until(() -> routeIndex >= route.size() - 1 && atTargets() && bothStopped())
            .withTimeout(SuperstructureConstants.COORD_TIMEOUT_SECONDS);
    }

    /**
     * Builds the move: plan a route through the free region from where the
     * mechanisms are, then follow it. If no route exists from the current
     * pose - which takes a pose only manual control can create, such as a
     * tucked arm parked high - the arm is first swung to the safe travel
     * angle, which is clear at every height, and the route is planned from
     * there.
     */
    private Command planMove(double targetHeight, double targetAngle) {
        double target = Math.max(targetHeight, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        double h0 = elevator.getCurrentPosition();
        double a0 = coral.getPivotAngle();

        List<double[]> route = planRoute(h0, a0, target, targetAngle);
        if (!route.isEmpty()) {
            return follow(route).andThen(settle());
        }
        return stream(() -> target, () -> SAFE_ANGLE,
                () -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE))
            .andThen(Commands.defer(() -> {
                List<double[]> retry = planRoute(elevator.getCurrentPosition(), coral.getPivotAngle(),
                    target, targetAngle);
                return retry.isEmpty() ? Commands.none() : follow(retry);
            }, Set.of(elevator, coral)))
            .andThen(settle());
    }

    // ==================================================================
    // Coral
    // ==================================================================

    /**
     * Stows to the base pose (elevator down, arm tucked to 0) with the
     * rollers stopped. This is also the pre-intake position.
     */
    public Command stow() {
        return setGoal(Goal.STOW)
            .andThen(Commands.runOnce(coral::stopIntake, coral))
            .andThen(moveTo(PresetHeights.BASE, PivotPresetAngles.BASE));
    }

    /**
     * Swings the arm to the safe travel angle from wherever the mechanisms
     * are, using the staged exits when leaving L3 or L4.
     */
    public Command raiseArm() {
        return Commands.defer(() -> {
            double hold = elevator.getCurrentPosition();
            return stream(() -> hold, () -> SAFE_ANGLE,
                    () -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE) && coral.isAtTargetAngle())
                .andThen(settle());
        }, Set.of(elevator, coral));
    }

    /**
     * Full coral intake: stow to the base pose, then run the rollers until
     * the CANrange confirms a game piece (the subsystem stops them
     * automatically on confirmation). Skipped entirely if a piece is
     * already held.
     */
    public Command intakeCoral() {
        return stow().andThen(intakeRollers())
            .unless(coral::isGamePieceDetected);
    }

    /**
     * Roller-only intake at the current pose: run until the CANrange
     * confirms a game piece, then stop. Skipped if a piece is already held.
     * Autos bound THIS with a timeout (after their own stow) - a timeout
     * around the whole intakeCoral would race the stow's settle timeout and
     * could expire before the rollers ever start.
     */
    public Command intakeRollers() {
        return Commands.sequence(
            Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.CORAL_INTAKE_SPEED), coral),
            Commands.waitUntil(coral::isGamePieceDetected),
            // Explicit stop: the subsystem's auto-stop only fires on the
            // RISING edge of detection - if the coral was already latched
            // by the time the rollers started (e.g. it arrived during the
            // stow phase), no edge ever comes and the rollers would run
            // forever without this.
            Commands.runOnce(coral::stopIntake, coral)
        ).handleInterrupt(coral::stopIntake) // Never leave rollers running on interrupt
            .unless(coral::isGamePieceDetected);
    }

    /** Moves to the L1 scoring pose (base height, 100 degrees). */
    public Command goToCoralL1() {
        return setGoal(Goal.CORAL_L1)
            .andThen(moveTo(PresetHeights.CORAL_L1, PivotPresetAngles.CORAL_L1));
    }

    /** Moves to the L2 scoring pose (12 in, 12.5 degrees). */
    public Command goToCoralL2() {
        return setGoal(Goal.CORAL_L2)
            .andThen(moveTo(PresetHeights.CORAL_L2, PivotPresetAngles.CORAL_L2));
    }

    /** Moves to the L3 scoring pose (30.5 in, 25 degrees). */
    public Command goToCoralL3() {
        return setGoal(Goal.CORAL_L3)
            .andThen(moveTo(PresetHeights.CORAL_L3, PivotPresetAngles.CORAL_L3));
    }

    /** Moves to the L4 scoring pose (52.5 in, 20 degrees). */
    public Command goToCoralL4() {
        return setGoal(Goal.CORAL_L4)
            .andThen(moveTo(PresetHeights.CORAL_L4, PivotPresetAngles.CORAL_L4));
    }

    /**
     * Ejects the held coral at the current pose: run the rollers outward
     * briefly, then stop them. The operator drives the pose; this only
     * handles the rollers.
     */
    public Command ejectCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.CORAL_SCORE_SPEED), coral),
            Commands.waitSeconds(0.5),
            Commands.runOnce(coral::stopIntake, coral)
        ).handleInterrupt(coral::stopIntake); // Never leave rollers running on interrupt
    }

    // ==================================================================
    // Algae
    // ==================================================================

    /** Moves to the low algae intake pose (20.5 in, 160 degrees) and runs the rollers inward. */
    public Command intakeAlgaeLow() {
        return setGoal(Goal.ALGAE_LOW)
            .andThen(moveTo(PresetHeights.ALGAE_LOW_INTAKE, PivotPresetAngles.ALGAE_INTAKE))
            .andThen(Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_INTAKE_SPEED), coral));
    }

    /** Moves to the high algae intake pose (37.5 in, 160 degrees) and runs the rollers inward. */
    public Command intakeAlgaeHigh() {
        return setGoal(Goal.ALGAE_HIGH)
            .andThen(moveTo(PresetHeights.ALGAE_HIGH_INTAKE, PivotPresetAngles.ALGAE_INTAKE))
            .andThen(Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_INTAKE_SPEED), coral));
    }

    /**
     * Holds an algae: light inward roller pressure and the arm at the hold
     * angle (RAISE = 100 degrees, the safe travel angle, so the elevator
     * stays free to move anywhere). The elevator is left where it is.
     */
    public Command holdAlgae() {
        return Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_HOLD_SPEED), coral)
            .andThen(raiseArm()); // Hold angle == safe travel angle
    }

    /**
     * Moves to the algae scoring pose (52.5 in, 105 degrees), ejects for
     * half a second, then stops the rollers (mirroring ejectCoral - without
     * the stop they would spin at 50% duty until another roller command).
     */
    public Command scoreAlgae() {
        return setGoal(Goal.ALGAE_SCORE)
            .andThen(moveTo(PresetHeights.ALGAE_SCORE, PivotPresetAngles.ALGAE_SCORE))
            .andThen(Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_SCORE_SPEED), coral))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.runOnce(coral::stopIntake, coral))
            .handleInterrupt(coral::stopIntake); // Never leave rollers running on interrupt
    }

    // ==================================================================
    // Independent mechanism testing (Elastic Testing tab)
    // ==================================================================
    // These commands move ONE mechanism and leave the other exactly where
    // it is, so each can be tuned and exercised on its own from the
    // dashboard. They consult the same CAD corridors as the planner - but
    // instead of moving the other mechanism out of the way, an unsafe
    // request is simply REFUSED with a dashboard notification explaining
    // why. Every command requires only the mechanism it moves, so the
    // operator's manual control of the other one keeps working.

    /** True if the elevator can move from its current height to the target with the arm held where it is. */
    public boolean isElevatorOnlyMoveSafe(double targetHeight) {
        return elevatorPathClear(elevator.getCurrentPosition(), targetHeight, coral.getPivotAngle());
    }

    /** True if the arm can rotate from its current angle to the target with the elevator held where it is. */
    public boolean isPivotOnlyMoveSafe(double targetAngle) {
        return pivotPathClear(coral.getPivotAngle(), targetAngle, elevator.getCurrentPosition());
    }

    /**
     * Elevator-only move to a dashboard-supplied height (inches). Refuses,
     * with a notification, if the arm's current angle makes the move unsafe.
     */
    public Command testElevatorTo(DoubleSupplier heightInches) {
        return Commands.defer(() -> {
            double target = heightInches.getAsDouble();
            if (!isElevatorOnlyMoveSafe(target)) {
                Elastic.sendNotification(new Elastic.Notification(
                    Elastic.NotificationLevel.WARNING,
                    "Elevator test move refused",
                    String.format("%.1f in is not reachable with the arm at %.0f deg (CAD contact band). "
                        + "Put the arm at %.0f deg (Pivot Setpoint %.0f + Pivot Go) for full travel.",
                        target, coral.getPivotAngle(), SAFE_ANGLE, SAFE_ANGLE)));
                return Commands.none();
            }
            return elevatorTo(target);
        }, Set.of(elevator));
    }

    /**
     * Pivot-only move to a dashboard-supplied angle (degrees). Refuses, with
     * a notification, if the elevator's current height makes the move unsafe.
     */
    public Command testPivotTo(DoubleSupplier angleDegrees) {
        return Commands.defer(() -> {
            double target = angleDegrees.getAsDouble();
            if (!isPivotOnlyMoveSafe(target)) {
                Elastic.sendNotification(new Elastic.Notification(
                    Elastic.NotificationLevel.WARNING,
                    "Pivot test move refused",
                    String.format("%.0f deg is not reachable with the elevator at %.1f in (CAD contact band). "
                        + "All of 25-100 deg is clear around 33 in; near the base, 8 deg and up is clear.",
                        target, elevator.getCurrentPosition())));
                return Commands.none();
            }
            return armTo(target);
        }, Set.of(coral));
    }

    /** Runs the intake rollers at a dashboard-supplied duty cycle (-1 to 1). */
    public Command testIntakeRun(DoubleSupplier dutyCycle) {
        return Commands.runOnce(() ->
            coral.setIntakeSpeed(Math.max(-1.0, Math.min(1.0, dutyCycle.getAsDouble()))), coral);
    }

    /** Stops the intake rollers. */
    public Command testIntakeStop() {
        return Commands.runOnce(coral::stopIntake, coral);
    }
}
