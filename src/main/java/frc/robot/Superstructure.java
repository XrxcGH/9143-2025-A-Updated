package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
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
 * the robot goes through this class, whose planner moves through that
 * region in staged, STATE-GATED steps: each step's trigger is a measured
 * height or angle, never a timer, so the sequences stay safe at any
 * elevator or pivot speed (only their duration changes).
 *
 * The shape of the free region, and what it forces:
 *   - Tucked (below ARM_CLEAR_MIN_ANGLE) only below ARM_TUCK_MAX_HEIGHT.
 *   - "Low box": any angle from ARM_CLEAR_MIN_ANGLE up is free below
 *     LOW_BOX_ROOF, so low poses (L2, L1) move both mechanisms together.
 *   - Climbing past the low box roof needs the arm at BAND_PASS_MIN_ANGLE
 *     or more; from SAFE_TRAVEL_MIN_ANGLE (RAISE = 100 deg) the carriage
 *     may go anywhere. 90 deg is NOT a safe travel angle above ~36 in.
 *   - Mid scoring poses (L3): the carriage climbs at RAISE and the arm
 *     makes its final rotation just below the target. Leaving: the
 *     carriage LIFTS to MID_POSE_RETURN_LIFT_HEIGHT while the arm swings
 *     up and only descends once the arm is back at RAISE - the fix for the
 *     L3 return hitting the middle-stage tube.
 *   - High scoring pose (L4): the top of travel is clear only with the arm
 *     at 22.5 deg or less, and the 90->L4 rotation is impossible up there,
 *     so the arm rotates at the L4_STATION_HEIGHT "station" (33 in, where
 *     25-100 deg are all clear), the carriage climbs with the arm at the
 *     stage angle, and the last few degrees happen above
 *     L4_FINAL_ANGLE_MIN_HEIGHT. Leaving L4 is the mirror: DROP first to
 *     L4_RETURN_DROP_HEIGHT at the L4 angle, stage to 45 deg, drop to the
 *     station, swing to RAISE, then descend.
 *   - Angles beyond HIGH_ANGLE_STAGE (algae poses) hit the bumper near the
 *     base, so the arm waits at the stage angle until the carriage is above
 *     HIGH_ANGLE_MIN_HEIGHT.
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

    /** One-line description of the staged-sequence stations, for the dashboard. */
    public static String stationSummary() {
        return String.format(
            "L4 up: RAISE to %.0f in, rotate to %.0f deg there, climb to %.0f, finish %.0f deg above %.0f, top | "
                + "L4 down: drop to %.0f at %.0f deg, %.0f deg below %.0f, station %.0f, RAISE below %.0f | "
                + "L3 up: rotate %.0f in below target; down: lift to %.0f, descend at RAISE | "
                + "L3 <-> L4: direct, arm turns inside the band overlap (no RAISE)",
            SuperstructureConstants.L4_STATION_HEIGHT, SuperstructureConstants.L4_STAGE_ANGLE,
            SuperstructureConstants.L4_PRE_TOP_HEIGHT, PivotPresetAngles.CORAL_L4.getAngle(),
            SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT,
            SuperstructureConstants.L4_RETURN_DROP_HEIGHT, PivotPresetAngles.CORAL_L4.getAngle(),
            SuperstructureConstants.L4_RETURN_STAGE_ANGLE, SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT,
            SuperstructureConstants.L4_STATION_HEIGHT, SuperstructureConstants.L4_RETURN_SAFE_ROTATE_MAX_HEIGHT,
            SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET, SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT);
    }

    // ==================================================================
    // Measured-state gates and single-mechanism steps
    // ==================================================================

    private boolean armAtLeast(double angle) {
        return coral.getPivotAngle() >= angle - TOL;
    }

    private boolean armAtMost(double angle) {
        return coral.getPivotAngle() <= angle + TOL;
    }

    /**
     * Strict form of {@link #armAtMost}, for a gate that is a CLEARANCE
     * limit rather than a "close enough to carry on" check. The tolerant
     * form accepts SAFE_ANGLE_TOLERANCE past the gate, which for the L4
     * pre-top gate (22.5 deg) means it opens at 25.5 - the very angle that
     * put the claw into the top bar. The arm's L4 destination is 20 deg, so
     * it passes 22.5 on the way and this cannot stall.
     */
    private boolean armStrictlyAtMost(double angle) {
        return coral.getPivotAngle() <= angle;
    }

    private boolean heightAtLeast(double height) {
        return elevator.getCurrentPosition() >= height;
    }

    private boolean heightAtMost(double height) {
        return elevator.getCurrentPosition() <= height;
    }

    private Command armTo(double angle) {
        return Commands.runOnce(() -> coral.setPivotAngle(angle), coral);
    }

    private Command elevatorTo(double height) {
        return Commands.runOnce(() -> elevator.setPosition(height), elevator);
    }

    private Command both(double height, double angle) {
        return Commands.runOnce(() -> {
            elevator.setPosition(height);
            coral.setPivotAngle(angle);
        }, elevator, coral);
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
     * mechanisms have started. Intermediate SAFETY gates deliberately do
     * not get this treatment.
     */
    private Command settle() {
        return Commands.waitSeconds(SuperstructureConstants.SETTLE_MIN_SECONDS)
            .andThen(Commands.waitUntil(() -> atTargets() || bothStopped()))
            .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS);
    }

    /**
     * Highest carriage height that is clear with the arm at the given
     * MEASURED angle while it swings up toward RAISE (used for the climbing
     * head start). No tolerance on the clear angle: the tuck zone is real.
     */
    private static double climbCeiling(double armAngle) {
        if (armAngle < SuperstructureConstants.ARM_CLEAR_MIN_ANGLE + SuperstructureConstants.SAFE_ANGLE_TOLERANCE) {
            return SuperstructureConstants.ARM_TUCK_MAX_HEIGHT;
        }
        if (armAngle < SuperstructureConstants.BAND_PASS_MIN_ANGLE) {
            return SuperstructureConstants.LOW_BOX_ROOF;
        }
        if (armAngle < SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE) {
            // A margin under the mid-corridor ceiling, so a carriage parked
            // here is inside the L4 rotation window, not resting on its edge.
            return SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT - SuperstructureConstants.RATCHET_MARGIN;
        }
        return ElevatorConstants.ELEVATOR_MAX_POSITION;
    }

    // ==================================================================
    // Continuous carriage targets ("ratchets")
    // ==================================================================
    // The staged sequences below are safe because every handover waits on
    // MEASURED state. What made them feel like a routine was the carriage
    // stopping at each intermediate height while the arm worked. These two
    // helpers remove those stops without touching the handover logic: the
    // carriage is given the FINAL height every loop, clamped to what the
    // corridors allow for the angles the arm still has to sweep, so it
    // climbs (or descends) continuously as the arm unlocks travel.
    //
    // This is cheap - one walk of the 32-row corridor table per loop, no
    // search - and it cannot deadlock, because the arm's schedule is
    // unchanged and its direction within a stage is known: the limit only
    // moves in the carriage's favour as the arm advances.

    private static final double NO_BAND = Double.NaN;

    /**
     * Ceiling of the corridor band containing {@code height} at
     * {@code angle}, or NaN when that pose is blocked. Allocation-free: this
     * runs every loop while a move is in progress.
     */
    private static double bandCeiling(double angle, double height) {
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            if (angle < row[0]) {
                if (height >= row[1] && height <= row[2]) {
                    return row[2];
                }
                if (!Double.isNaN(row[3]) && height >= row[3] && height <= row[4]) {
                    return row[4];
                }
                return NO_BAND;
            }
        }
        return NO_BAND;
    }

    /** Floor of the corridor band containing {@code height} at {@code angle}, or NaN when blocked. */
    private static double bandFloor(double angle, double height) {
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            if (angle < row[0]) {
                if (height >= row[1] && height <= row[2]) {
                    return row[1];
                }
                if (!Double.isNaN(row[3]) && height >= row[3] && height <= row[4]) {
                    return row[3];
                }
                return NO_BAND;
            }
        }
        return NO_BAND;
    }

    /**
     * Highest height that stays clear for every angle between {@code from}
     * and {@code to} - the angles the arm still has to pass through - so the
     * carriage never climbs into a height the arm is about to block.
     */
    /*
     * FAILS CLOSED. The walk stops at the first angle whose band does not
     * contain this height. That is legitimate when moving on is what
     * unlocks that angle - its NEAREST band is ahead of the carriage (the
     * L4 climb: 20 deg only opens above 35.5 in, 2.5 in ahead). It is NOT
     * legitimate when the arm is about to ENTER that angle (it is within
     * IMMINENT_SWEEP of where the arm is now) and the row's nearest band is
     * BEHIND the carriage - it overshot that row's ceiling - or when the
     * current pose is outside the table altogether. The old walk returned
     * "no limit" for both, releasing the carriage exactly when it should
     * hold; those now return the current height. A blocked row far along
     * the sweep is still just the end of the walk: the arm is gated on the
     * carriage's height before it gets there.
     */
    public static double ceilingForSweep(double from, double to, double height) {
        double ceiling = ElevatorConstants.ELEVATOR_MAX_POSITION;
        double start = snapIntoTable(from, height);
        double step = to >= start ? SWEEP_STEP : -SWEEP_STEP;
        double angle = start;
        boolean first = true;
        while (true) {
            double rowCeiling = bandCeiling(angle, height);
            if (Double.isNaN(rowCeiling)) {
                if (first || (Math.abs(angle - start) <= IMMINENT_SWEEP && !rowOpensToward(angle, height, true))) {
                    return Math.min(ceiling, height); // hold: do not climb into it
                }
                break;
            }
            first = false;
            ceiling = Math.min(ceiling, rowCeiling);
            if (angle == to) {
                break;
            }
            angle += step;
            if ((step > 0 && angle > to) || (step < 0 && angle < to)) {
                angle = to;
            }
        }
        return ceiling;
    }

    /** Mirror of {@link #ceilingForSweep} for descents. */
    public static double floorForSweep(double from, double to, double height) {
        double floor = ElevatorConstants.ELEVATOR_ZERO_HEIGHT;
        double start = snapIntoTable(from, height);
        double step = to >= start ? SWEEP_STEP : -SWEEP_STEP;
        double angle = start;
        boolean first = true;
        while (true) {
            double rowFloor = bandFloor(angle, height);
            if (Double.isNaN(rowFloor)) {
                if (first || (Math.abs(angle - start) <= IMMINENT_SWEEP && !rowOpensToward(angle, height, false))) {
                    return Math.max(floor, height); // hold: do not descend into it
                }
                break;
            }
            first = false;
            floor = Math.max(floor, rowFloor);
            if (angle == to) {
                break;
            }
            angle += step;
            if ((step > 0 && angle > to) || (step < 0 && angle < to)) {
                angle = to;
            }
        }
        return floor;
    }

    private static final double SWEEP_STEP = 2.5;
    /** A blocked row this close (deg) to the arm is one the arm is about to enter. */
    private static final double IMMINENT_SWEEP = 5.0;

    /**
     * The measured arm angle, nudged by at most SAFE_ANGLE_TOLERANCE onto a
     * table row that contains this height. A scoring pose that sits ON a
     * row boundary (L3 is 25.0 deg, the edge of the 25-30 row whose
     * corridor starts at 30 in) reads 24.x on the through bore once the
     * chain settles, and the 20-25 row says 30.5 in is blocked - so every
     * lookup from the resting L3 pose failed, and the L3 -> L4 shortcut was
     * never taken. The exposure is the one the resting pose already has.
     */
    static double snapIntoTable(double angle, double height) {
        if (!Double.isNaN(bandCeiling(angle, height))) {
            return angle;
        }
        for (double d = 1.0; d <= SuperstructureConstants.SAFE_ANGLE_TOLERANCE + 1e-9; d += 1.0) {
            if (!Double.isNaN(bandCeiling(angle + d, height))) {
                return angle + d;
            }
            if (!Double.isNaN(bandCeiling(angle - d, height))) {
                return angle - d;
            }
        }
        return angle;
    }

    /**
     * True if the band of this angle's row NEAREST to {@code height} lies on
     * the travel side of it (above when climbing): carrying on is what
     * unlocks that angle. False when the nearest band is behind - the
     * carriage has overshot it - or the row has no band at all.
     */
    private static boolean rowOpensToward(double angle, double height, boolean up) {
        for (double[] row : SuperstructureConstants.FREE_CORRIDORS) {
            if (angle < row[0]) {
                double nearest = Double.POSITIVE_INFINITY;
                boolean nearestIsAhead = false;
                for (int b = 1; b <= 3; b += 2) {
                    if (Double.isNaN(row[b])) {
                        continue;
                    }
                    boolean above = row[b] > height;
                    double gap = above ? row[b] - height : height - row[b + 1];
                    boolean ahead = above == up;
                    if (gap < nearest || (gap == nearest && ahead)) {
                        nearest = gap;
                        nearestIsAhead = ahead;
                    }
                }
                return nearestIsAhead;
            }
        }
        return false;
    }

    // ------------------------------------------------------------------
    // Carriage setpoint latch
    // ------------------------------------------------------------------
    // MAXMotion restarts its profile from the MEASURED position and a
    // lagged measured velocity every time the setpoint changes. The ratchets
    // used to send max(clamp, measuredHeight) every loop, so at a clamp the
    // setpoint changed every 20 ms and the profile restarted fifty times a
    // second - the hunting. The latch sends a new setpoint only when the
    // clamp really moved, and holds off small steps while the carriage is
    // still far from needing them.

    /** Smallest clamp advance worth a profile restart while the carriage is not yet braking for the old one. */
    private static final double LATCH_MIN_STEP = 2.0;   // inches
    /** Extra distance on top of the braking distance inside which a clamp advance is sent at once. */
    private static final double LATCH_BRAKE_PAD = 1.5;  // inches
    /** A clamp that moves BACK by less than this is ignored (arm-angle chatter at a gate). */
    private static final double LATCH_RETREAT_MIN = 0.25; // inches

    /**
     * Sends {@code desired} to the carriage through a per-command latch
     * ({@code latch[0]}, NaN until first use). {@code up} is the travel
     * direction the desired value was computed for.
     */
    private void commandCarriage(double[] latch, double desired, double finalTarget, boolean up) {
        double height = elevator.getCurrentPosition();
        if (Double.isNaN(latch[0])) {
            latch[0] = up ? Math.max(desired, height) : Math.min(desired, height);
            elevator.setPosition(latch[0]);
            return;
        }
        double advance = up ? desired - latch[0] : latch[0] - desired;
        if (advance > 1e-6) {
            double v = elevator.getVelocity();
            double braking = v * v / (2.0 * Math.max(elevator.maxAcceleration(), 1.0)) + LATCH_BRAKE_PAD;
            boolean reachesTarget = up ? desired >= finalTarget - 1e-6 : desired <= finalTarget + 1e-6;
            boolean aboutToBrake = Math.abs(latch[0] - height) <= braking;
            if (reachesTarget || advance >= LATCH_MIN_STEP || aboutToBrake) {
                latch[0] = desired;
                elevator.setPosition(desired);
            }
        } else if (advance < -LATCH_RETREAT_MIN) {
            // The clamp moved back (a gate re-closed). Stop where we are,
            // once, rather than chasing the measured height every loop.
            double stopAt = up ? Math.max(desired, height) : Math.min(desired, height);
            boolean stillAhead = up ? latch[0] - stopAt > LATCH_RETREAT_MIN : stopAt - latch[0] > LATCH_RETREAT_MIN;
            if (stillAhead) {
                latch[0] = stopAt;
                elevator.setPosition(stopAt);
            }
        }
    }

    /**
     * Climbs toward {@code target} as fast as the arm allows: every loop the
     * carriage is given the target, clamped below the ceiling for the sweep
     * the arm has left to do, and never below where it already is.
     */
    private Command climbWithArm(DoubleSupplier targetSupplier, DoubleSupplier armDestination) {
        double[] latch = {Double.NaN};
        return Commands.run(() -> {
            double target = targetSupplier.getAsDouble();
            commandCarriage(latch, climbGoal(target, armDestination.getAsDouble()), target, true);
        }, elevator);
    }

    /** The height a climb toward {@code target} may be commanded to right now. */
    private double climbGoal(double target, double armDestination) {
        double raw = ceilingForSweep(coral.getPivotAngle(), armDestination, elevator.getCurrentPosition());
        // A target the corridor allows is commanded as it is - the scoring
        // poses sit near the top of their band by design, so subtracting a
        // margin from those would leave the carriage permanently low. Only
        // a target the corridor does NOT allow is clamped, and then it stops
        // a margin short of the limit rather than exactly on it, so the
        // tracking error of a carriage decelerating into its clamp stays
        // inside the corridor.
        return target <= raw ? target : raw - SuperstructureConstants.RATCHET_MARGIN;
    }

    /** Mirror of {@link #climbGoal}: a legal target as it is, an illegal one a margin ABOVE the floor. */
    private double descentGoal(double target, double armDestination) {
        double raw = floorForSweep(coral.getPivotAngle(), armDestination, elevator.getCurrentPosition());
        return target >= raw ? target : raw + SuperstructureConstants.RATCHET_MARGIN;
    }

    /**
     * Travels toward {@code target} in whichever direction it lies, as fast
     * as the arm's sweep allows - the climb ratchet when the target is
     * above, the descent ratchet when it is below. Used wherever the arm has
     * work to do and the carriage would otherwise sit still waiting for it.
     */
    private Command travelWithArm(DoubleSupplier targetSupplier, DoubleSupplier armDestination) {
        double[] latch = {Double.NaN};
        boolean[] wasUp = {true};
        return Commands.run(() -> {
            double target = targetSupplier.getAsDouble();
            // Direction from the last COMMANDED height once there is one, so
            // a carriage resting a hair either side of its setpoint does not
            // flip the ratchet back and forth.
            double reference = Double.isNaN(latch[0]) ? elevator.getCurrentPosition() : latch[0];
            boolean up = target >= reference;
            if (up != wasUp[0]) {
                latch[0] = Double.NaN; // the target moved to the other side: start a fresh latch
                wasUp[0] = up;
            }
            double goal = up ? climbGoal(target, armDestination.getAsDouble())
                : descentGoal(target, armDestination.getAsDouble());
            commandCarriage(latch, goal, target, up);
        }, elevator);
    }

    // ==================================================================
    // Direct transfer between scoring poses
    // ==================================================================

    /**
     * Height at which the arm can turn straight from {@code fromAngle} to
     * {@code toAngle} on the way between two poses, or NaN when no such
     * height exists and the move has to go round by RAISE.
     *
     * Two poses whose corridor bands OVERLAP do not need the RAISE
     * excursion. L3 (25 deg, clear 30-51 in) and L4 (20 deg, clear
     * 35.5-52.5 in) overlap over 36-51, so anywhere in there the arm simply
     * turns the five degrees between them; the generic plan rotated 75 deg
     * out to RAISE and 80 back to avoid those five.
     *
     * Checked against every ordered pair of operator poses, this is the only
     * pair it changes, and that is geometry rather than luck:
     *   - BASE/L2 to L3/L4 - the low box (up to ~20 in) and the upper
     *     corridor (from ~30 in) do not overlap at any low angle, because
     *     the middle-stage top tube sits between them. The arm HAS to come
     *     up to 75-100 deg to cross, so the excursion is the move.
     *   - anything to L1, the algae poses, or from them - those poses are at
     *     or beyond RAISE already, so there is no excursion to remove; the
     *     existing plan commands both mechanisms together.
     * That leaves L3 and L4, the two poses that share the upper corridor.
     *
     * The turn happens at the point of the overlap nearest the carriage, so
     * it starts as soon as the carriage gets there and never detours to
     * reach it. It is never above the pre-top height: the table is
     * optimistic at the top of travel (this is where L4 caught the top bar)
     * and the pre-top height is the ceiling the robot has actually held the
     * carriage at while the arm swept through these angles.
     */
    public static double directTransferHeight(double fromHeight, double fromAngle,
            double toHeight, double toAngle) {
        double target = Math.max(toHeight, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        // Scoring angles only. At or past the band-pass angle the arm is
        // already free to travel, so the normal plan has no excursion in it.
        if (fromAngle >= SuperstructureConstants.BAND_PASS_MIN_ANGLE
                || toAngle >= SuperstructureConstants.BAND_PASS_MIN_ANGLE) {
            return NO_BAND;
        }
        // The MEASURED start angle, nudged onto the row that holds this
        // height: a resting L3 reads 24.x deg, a hair under its own row.
        fromAngle = snapIntoTable(fromAngle, fromHeight);
        double fromLo = bandFloor(fromAngle, fromHeight);
        double toLo = bandFloor(toAngle, target);
        if (Double.isNaN(fromLo) || Double.isNaN(toLo)) {
            return NO_BAND; // one of the poses is not in a corridor at all
        }
        double lo = Math.max(fromLo, toLo) + SuperstructureConstants.RATCHET_MARGIN;
        double hi = Math.min(bandCeiling(fromAngle, fromHeight), bandCeiling(toAngle, target))
            - SuperstructureConstants.RATCHET_MARGIN;
        hi = Math.min(hi, SuperstructureConstants.L4_PRE_TOP_HEIGHT);
        if (lo > hi) {
            return NO_BAND; // the bands do not overlap: go round by RAISE
        }
        double rotateAt = Math.min(Math.max(fromHeight, lo), hi);
        if (!pivotPathClear(fromAngle, toAngle, rotateAt)
                || !elevatorPathClear(fromHeight, rotateAt, fromAngle)
                || !elevatorPathClear(rotateAt, target, toAngle)) {
            return NO_BAND;
        }
        return rotateAt;
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

    /** Builds the motion sequence for a target pose from the current state. */
    private Command planMove(double targetHeight, double targetAngle) {
        double h0 = elevator.getCurrentPosition();
        double a0 = coral.getPivotAngle();
        double target = Math.max(targetHeight, ElevatorConstants.ELEVATOR_ZERO_HEIGHT);

        // ---- Moves that never leave the low box need no RAISE excursion ----
        // "Starts low" means it will STILL be low once it has stopped: a
        // second press can catch the carriage at 15 in climbing at 40 in/s,
        // and that carriage coasts four inches through the low box roof.
        double v0 = elevator.getVelocity();
        double coast = v0 > 0 ? v0 * v0 / (2.0 * Math.max(elevator.maxAcceleration(), 1.0)) : 0.0;
        boolean startsLow = h0 + coast <= SuperstructureConstants.LOW_BOX_ROOF;
        boolean endsLow = target <= SuperstructureConstants.LOW_BOX_ROOF;
        boolean armStartsClear = a0 >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE;
        if (startsLow && endsLow && targetAngle < SuperstructureConstants.HIGH_ANGLE_STAGE) {
            if (targetAngle < SuperstructureConstants.ARM_CLEAR_MIN_ANGLE) {
                // Tuck target (BASE): descend at the current angle (the low
                // box is clear at any angle from ARM_CLEAR_MIN_ANGLE up),
                // tuck once below the tuck limit. From the base pose this is
                // (almost) a no-op instead of a 0 -> 100 -> 0 round trip.
                // A tucked arm ABOVE the tuck limit is a manual-control-only
                // state: swing it clear first, in place.
                Command clearFirst = (!armStartsClear && h0 > SuperstructureConstants.ARM_TUCK_MAX_HEIGHT)
                    ? armTo(SuperstructureConstants.ARM_CLEAR_MIN_ANGLE + TOL)
                        .andThen(Commands.waitUntil(() -> coral.getPivotAngle()
                            >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE))
                    : Commands.none();
                return clearFirst
                    .andThen(elevatorTo(target))
                    .andThen(Commands.waitUntil(() -> heightAtMost(SuperstructureConstants.ARM_TUCK_MAX_HEIGHT)))
                    .andThen(armTo(targetAngle))
                    .andThen(settle());
            }
            // Low pose (L2, L1): arm first if it starts tucked (it only has
            // to clear ~8 degrees before the carriage may leave the base),
            // then both together.
            return armTo(targetAngle)
                .andThen(Commands.waitUntil(() -> coral.getPivotAngle()
                    >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE || a0 >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE))
                .andThen(elevatorTo(target))
                .andThen(settle());
        }

        // ---- General path: get the arm to RAISE (with every head start the
        //      corridors allow), then approach the target ----
        // An arm beyond the high-angle stage (the algae poses, 160 deg) may
        // not be that far over below HIGH_ANGLE_MIN_HEIGHT (bumper /
        // Limelight bracket), so before any target that is not itself a
        // free-zone pose at height, bring the arm back to RAISE (or the
        // free-zone target angle) while the carriage holds at or above the
        // minimum height - then plan as usual from there.
        Command lowerFirst = Commands.none();
        if (a0 > SuperstructureConstants.HIGH_ANGLE_STAGE
                && (targetAngle < SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE
                    || target < SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT)) {
            double holdAngle = targetAngle >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE ? targetAngle : SAFE_ANGLE;
            lowerFirst = both(Math.max(target, SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT), holdAngle)
                .andThen(Commands.waitUntil(() -> armAtMost(SuperstructureConstants.HIGH_ANGLE_STAGE)));
        }
        // ---- Scoring pose to scoring pose (L3 <-> L4) ----
        // Their corridors overlap, so the arm turns the few degrees between
        // them at a height that suits both instead of swinging out to RAISE
        // and back. Every other pair either has no overlap (the tube between
        // the low box and the upper corridor) or is already at a travel
        // angle, and falls through to the plan below.
        double rotateAt = directTransferHeight(h0, a0, target, targetAngle);
        if (!Double.isNaN(rotateAt)) {
            return directTransfer(h0, rotateAt, target, targetAngle);
        }
        return lowerFirst.andThen(escapeToSafe(target)).andThen(approach(target, targetAngle));
    }

    /**
     * Runs a direct scoring-pose transfer: travel to the turn height, and
     * turn there while the carriage carries on toward the target behind the
     * ratchet. The carriage is moving the whole time, and above the pre-top
     * height it still waits for the arm's final gate - the one limit the
     * robot, rather than the table, put there.
     *
     * The last leg is commanded once the arm has ARRIVED, because a pose
     * whose band floor is a row boundary (L3 at exactly 25 deg) is only
     * reachable with the arm at its angle; the ratchet alone would hold the
     * carriage at the previous row's floor.
     */
    private Command directTransfer(double startHeight, double rotateAt, double target, double targetAngle) {
        // The turn starts when the carriage REACHES the turn height from the
        // side it started on - not within a window around it. The turn
        // height sits inside the overlap of the two bands with the ratchet
        // margin to spare, but a window around it does not: a symmetric one
        // would let the arm start turning an inch short of the overlap,
        // which for the L3 climb is below the 20-25 deg corridor floor.
        //
        // The carriage is NEVER sent to the turn height. It used to be, and
        // the gate then waited for the carriage to reach the very height its
        // profile was stopping at: a carriage resting 0.02 in short never
        // opened it, there was no timeout, and even when it opened the
        // carriage had stopped first. Now the carriage travels toward the
        // real target behind the ratchet from the first loop, and the turn
        // height is only the threshold the ARM waits for as it goes past.
        Command reachTurn = rotateAt >= startHeight
            ? Commands.waitUntil(() -> heightAtLeast(rotateAt))
            : Commands.waitUntil(() -> heightAtMost(rotateAt));
        boolean gated = target > SuperstructureConstants.L4_PRE_TOP_HEIGHT;
        DoubleSupplier carriageTarget = () ->
            gated && !armStrictlyAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)
                ? SuperstructureConstants.L4_PRE_TOP_HEIGHT : target;
        Command armWork = reachTurn
            .andThen(armTo(targetAngle))
            .andThen(armArrived());
        return Commands.deadline(armWork, travelWithArm(carriageTarget, () -> targetAngle))
            // Last leg once the arm has arrived (see above) - but never past
            // the pre-top height unless the arm is really inside its gate.
            .andThen(Commands.either(elevatorTo(target), Commands.none(),
                () -> !gated || armStrictlyAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)))
            .andThen(settle());
    }

    // ==================================================================
    // Arm-side clamp ("the other half of the ratchet")
    // ==================================================================
    // The carriage has always been clamped by where the ARM is
    // (ceilingForSweep / floorForSweep). The arm was only ever GATED: it sat
    // at RAISE until the carriage reached a window where its whole sweep was
    // clear at once (31-36 in), which is why the carriage arrived at its
    // 35.5 in clamp with the arm barely moving and stopped there, and why
    // the L3 rotation waited for the carriage to finish. With the arm clamped
    // by where the CARRIAGE is - the mirror image, one walk of the same table
    // per loop - the arm can start as soon as its first rows are clear and is
    // simply held at the edge of any row the carriage has not opened yet.
    // Each mechanism is limited by the other's MEASURED position, so the
    // pair stays inside the corridors at any relative speed, including one
    // of them stalling.

    private static int rowIndex(double angle) {
        double[][] rows = SuperstructureConstants.FREE_CORRIDORS;
        for (int k = 0; k < rows.length; k++) {
            if (angle < rows[k][0]) {
                return k;
            }
        }
        return -1;
    }

    /** True if one of the row's bands holds {@code height} with {@code margin} to spare at each end (a band that starts at the hard stop needs none there). */
    private static boolean rowHolds(double[] row, double height, double margin) {
        for (int b = 1; b <= 3; b += 2) {
            if (Double.isNaN(row[b])) {
                continue;
            }
            boolean aboveFloor = row[b] <= 0.0 ? height >= row[b] : height >= row[b] + margin;
            if (aboveFloor && height <= row[b + 1] - margin) {
                return true;
            }
        }
        return false;
    }

    /**
     * How far toward {@code target} the arm may be commanded from
     * {@code angle} with the carriage at {@code height}: the target itself
     * when every table row on the way holds this height (with
     * ARM_CLAMP_HEIGHT_MARGIN), otherwise ARM_CLAMP_ANGLE_MARGIN short of the
     * first row that does not. A pose that is outside the table altogether
     * holds the arm where it is. Allocation-free table walk.
     */
    public static double armLimitForHeight(double angle, double target, double height) {
        double[][] rows = SuperstructureConstants.FREE_CORRIDORS;
        double start = snapIntoTable(angle, height);
        int from = rowIndex(start);
        int to = rowIndex(target);
        if (from < 0 || to < 0 || !rowHolds(rows[from], height, 0.0)) {
            return angle;
        }
        int step = to < from ? -1 : 1;
        for (int j = from; j != to;) {
            j += step;
            if (!rowHolds(rows[j], height, SuperstructureConstants.ARM_CLAMP_HEIGHT_MARGIN)) {
                // The near edge of the blocked row: its upper bound going
                // down, the previous row's upper bound going up.
                return step < 0
                    ? rows[j][0] + SuperstructureConstants.ARM_CLAMP_ANGLE_MARGIN
                    : rows[j - 1][0] - SuperstructureConstants.ARM_CLAMP_ANGLE_MARGIN;
            }
        }
        return target;
    }

    /**
     * Drives the arm toward {@code targetAngle} as far as the carriage's
     * height allows, every loop, until it has arrived there (at target or
     * stopped - see {@link #armArrived}). {@code extraLimit} is applied on
     * top of the table (NaN = none): the robot-proven gates the table does
     * not know about. CorAl.setPivotAngle ignores a repeat of the target it
     * already has, so the TalonFX only sees a new request when the limit
     * actually moves - a handful of times per sweep, each a smooth Motion
     * Magic retarget.
     */
    private Command armWithCarriage(double targetAngle, DoubleSupplier extraLimit) {
        double[] finalSince = {Double.NaN};
        return Commands.run(() -> {
            double angle = coral.getPivotAngle();
            double limit = armLimitForHeight(angle, targetAngle, elevator.getCurrentPosition());
            double extra = extraLimit.getAsDouble();
            if (!Double.isNaN(extra)) {
                limit = targetAngle < angle ? Math.max(limit, extra) : Math.min(limit, extra);
            }
            coral.setPivotAngle(limit);
            if (limit != targetAngle) {
                finalSince[0] = Double.NaN;
            } else if (Double.isNaN(finalSince[0])) {
                finalSince[0] = Timer.getFPGATimestamp();
            }
        }, coral).until(() -> {
            if (Double.isNaN(finalSince[0])) {
                return false; // still held by the carriage: a safety wait, no timeout
            }
            double sinceFinal = Timer.getFPGATimestamp() - finalSince[0];
            return sinceFinal >= SuperstructureConstants.SETTLE_TIMEOUT_SECONDS
                || (sinceFinal >= SuperstructureConstants.ARM_ARRIVED_MIN_SECONDS
                    && (coral.isAtTargetAngle()
                        || Math.abs(coral.getPivotVelocity()) <= SuperstructureConstants.SETTLE_STOPPED_PIVOT_DEG_S));
        });
    }

    /**
     * Waits for the arm to arrive: inside its at-target window, OR stopped.
     * The window is measured on the through bore while the loop closes on
     * the rotor, so with chain slack the arm can rest a degree or two
     * outside it for good - and the old wait then parked BOTH mechanisms
     * for the whole 3 s timeout. Once the arm has stopped, waiting longer
     * cannot improve the angle.
     */
    private Command armArrived() {
        return Commands.waitSeconds(SuperstructureConstants.ARM_ARRIVED_MIN_SECONDS)
            .andThen(Commands.waitUntil(() -> coral.isAtTargetAngle()
                || Math.abs(coral.getPivotVelocity()) <= SuperstructureConstants.SETTLE_STOPPED_PIVOT_DEG_S))
            .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS);
    }

    /**
     * Gets the arm to RAISE from wherever the mechanisms are now, at a
     * height from which the carriage can travel freely, using the staged
     * exits for the L3/L4 poses and a climbing head start from the low box.
     * Ends with the arm at/above SAFE_TRAVEL_MIN_ANGLE. Never times out:
     * these are safety gates.
     */
    private Command escapeToSafe(double targetHeight) {
        double h0 = elevator.getCurrentPosition();
        double a0 = coral.getPivotAngle();

        if (a0 >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE - TOL) {
            // Free to travel - IF the arm is staying here. A second button
            // press mid-move finds the arm at 99 deg on its way down to a
            // scoring angle (or on its way out to 160), and the old plan's
            // setpoint is still latched on the TalonFX: nothing re-commanded
            // the arm, so it carried on while the new plan drove the carriage
            // through a band that is only clear at RAISE. Re-latch it.
            double armGoal = coral.getTargetAngle();
            boolean parked = armGoal >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE - TOL
                && armGoal <= SuperstructureConstants.HIGH_ANGLE_STAGE;
            if (parked || a0 > SuperstructureConstants.HIGH_ANGLE_STAGE) {
                return Commands.none(); // (beyond the stage angle, lowerFirst / approach re-command it)
            }
            return armTo(SAFE_ANGLE)
                .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE)
                    && coral.getPivotVelocity() >= -SuperstructureConstants.SETTLE_STOPPED_PIVOT_DEG_S));
        }
        if (h0 >= SuperstructureConstants.L4_ZONE_MIN_HEIGHT && a0 < SuperstructureConstants.BAND_PASS_MIN_ANGLE) {
            return leaveHighPose(targetHeight);
        }
        if (h0 > SuperstructureConstants.LOW_BOX_ROOF && a0 < SuperstructureConstants.BAND_PASS_MIN_ANGLE) {
            return leaveMidPose(h0, targetHeight);
        }
        if (h0 > SuperstructureConstants.LOW_BOX_ROOF) {
            // Arm between the band-pass and safe-travel angles above the low
            // box (e.g. 90 deg at 38 in after manual control): rotating on
            // toward RAISE moves the claw's rear away from the tube. The
            // carriage travels toward the target underneath it rather than
            // waiting, clamped to what the arm's measured angle allows.
            return Commands.deadline(
                armTo(SAFE_ANGLE)
                    .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE))),
                travelWithArm(() -> targetHeight, () -> SAFE_ANGLE));
        }

        // In the low box. Swing up; when the move is upward, let the carriage
        // climb as far as the arm's MEASURED angle allows, re-evaluated every
        // loop so the ceiling ratchets up as the arm swings (tuck limit ->
        // low box roof -> mid corridor -> anything). MAXMotion re-profiles
        // each retarget from the current motion state, so the carriage keeps
        // climbing smoothly. Never commands downward.
        // Swing to RAISE, and travel toward the target the whole time: up
        // behind the ratcheting ceiling (tuck limit -> low box roof -> mid
        // corridor -> anything), or down behind the falling floor. Either
        // way the carriage is moving while the arm is, so nothing waits.
        double[] climbLatch = {Double.NaN};
        return Commands.deadline(
            armTo(SAFE_ANGLE)
                .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE))),
            targetHeight > h0
                ? Commands.run(() -> commandCarriage(climbLatch,
                    Math.min(targetHeight, climbCeiling(coral.getPivotAngle())), targetHeight, true), elevator)
                : travelWithArm(() -> targetHeight, () -> SAFE_ANGLE));
    }

    /**
     * Leaves a mid-height scoring pose (L3): lift to the return-lift height
     * while the arm swings to RAISE, and only hand the carriage on once the
     * arm is free to travel. Descending any earlier sweeps the claw into
     * the middle-stage top tube.
     */
    private Command leaveMidPose(double currentHeight, double targetHeight) {
        boolean descendingNext = targetHeight < SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT;
        double lift = Math.max(currentHeight, SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT);
        // A descent may start as soon as the arm clears band A (the mid
        // corridor is clear all the way down from 75 deg up); a climb has
        // to wait for RAISE because 75-95 deg is blocked above ~36 in.
        double release = descendingNext ? SuperstructureConstants.BAND_PASS_MIN_ANGLE
            : SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE;
        // The carriage lifts clear of the tube while the arm starts up, then
        // descends as soon as the arm's angle allows it - the floor falls
        // from 30 in to nothing as the arm passes 75 deg - so the lift and
        // the descent are one motion rather than two.
        Command armWork = armTo(SAFE_ANGLE)
            .andThen(Commands.waitUntil(() -> armAtLeast(release)));
        // Once band A is cleared the carriage heads for the REAL target, up
        // or down. It used to head for the hard stop regardless, so a move
        // up from L3 (and raiseArm / holdAlgae) dived toward the base first.
        return Commands.deadline(armWork,
            travelWithArm(() -> armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE) ? targetHeight : lift,
                () -> SAFE_ANGLE));
    }

    /**
     * Leaves the high scoring pose (L4): drop first at the L4 angle, stage
     * the arm to 45 deg while low enough, drop to the station, swing to
     * RAISE below the safe-rotate height, then the carriage is free.
     */
    private Command leaveHighPose(double targetHeight) {
        boolean descendingNext = targetHeight < SuperstructureConstants.L4_STATION_HEIGHT;
        double release = descendingNext ? SuperstructureConstants.BAND_PASS_MIN_ANGLE
            : SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE;
        // The carriage drops first (the top is only clear at the L4 angle),
        // and once it is low enough for the arm to start swinging back, the
        // two run together: the arm sweeps up to RAISE while the carriage
        // keeps descending, its floor falling as the arm rises (39 in at 45
        // deg, 0 past 75). One continuous drop instead of drop-rotate-drop.
        // The arm used to be sent to 45 deg, decelerate into it, and only
        // then be sent on to RAISE - a velocity trough in the middle of the
        // swing - and that second leg had no height gate at all. It now
        // heads for RAISE in one go, clamped every loop by the carriage's
        // height (45-50 deg is open at 43 in, 70 at 39, RAISE below 36).
        Command armWork = Commands.deadline(
            Commands.waitUntil(() -> armAtLeast(release)),
            armWithCarriage(SAFE_ANGLE, () -> Double.NaN));
        // Mirror of the climb: continuous down to the station, and below it
        // only once the arm has cleared band A, which is what the staged
        // version waited for.
        // One command owns the carriage for the whole exit, so there is no
        // loop in which it sits on a stale setpoint between steps. Below the
        // station it heads for the REAL target (it used to head for the hard
        // stop even when the next pose was above).
        Command gatedArmWork = Commands.waitUntil(
                () -> heightAtMost(SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT))
            .andThen(armWork);
        return Commands.deadline(gatedArmWork,
            travelWithArm(() -> {
                if (armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE)) {
                    return targetHeight;                                   // band A cleared: on to the target
                }
                if (armAtLeast(SuperstructureConstants.L4_RETURN_STAGE_DONE_ANGLE)) {
                    return SuperstructureConstants.L4_STATION_HEIGHT;      // staged: the station (load-bearing)
                }
                return SuperstructureConstants.L4_RETURN_DROP_HEIGHT;      // arm still near the L4 angle
            }, () -> SAFE_ANGLE));
    }

    /**
     * Travels to the target and settles at the target angle, given the arm
     * is at/above the safe travel angle. Picks the strategy by target type.
     */
    private Command approach(double targetHeight, double targetAngle) {
        // Tuck target (BASE) from above: descend at RAISE, and start the arm
        // down as soon as the carriage is inside the low box - every angle
        // from ARM_CLEAR_MIN_ANGLE up is clear there, so the arm can come
        // most of the way round while the carriage is still descending
        // instead of waiting for it. The last few degrees into the tuck wait
        // for the tuck limit, which is the one part that is not clear higher.
        if (targetAngle < SuperstructureConstants.ARM_CLEAR_MIN_ANGLE) {
            return elevatorTo(targetHeight)
                .andThen(Commands.waitUntil(() -> heightAtMost(SuperstructureConstants.LOW_BOX_ROOF)))
                .andThen(armTo(SuperstructureConstants.ARM_CLEAR_MIN_ANGLE + TOL))
                .andThen(Commands.waitUntil(() -> heightAtMost(SuperstructureConstants.ARM_TUCK_MAX_HEIGHT)))
                .andThen(armTo(targetAngle))
                .andThen(settle());
        }
        // Free-zone target (RAISE .. stage angle, e.g. L1, algae score): together
        if (targetAngle >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE
                && targetAngle <= SuperstructureConstants.HIGH_ANGLE_STAGE) {
            return both(targetHeight, targetAngle).andThen(settle());
        }
        // High-angle target (algae intake): hold the arm at the stage angle
        // until the carriage is above the bumper zone, then finish. Already
        // above it (and staying there): go straight to the pose.
        if (targetAngle > SuperstructureConstants.HIGH_ANGLE_STAGE) {
            if (elevator.getCurrentPosition() >= SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT
                    && targetHeight >= SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT) {
                return both(targetHeight, targetAngle).andThen(settle());
            }
            return both(targetHeight, SuperstructureConstants.HIGH_ANGLE_STAGE)
                .andThen(Commands.waitUntil(() -> heightAtLeast(SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT)))
                .andThen(armTo(targetAngle))
                .andThen(settle());
        }
        // Low pose reached from above (L2, L1 at a low angle): descend at
        // RAISE and rotate only once inside the low box
        if (targetHeight <= SuperstructureConstants.LOW_BOX_ROOF) {
            return elevatorTo(targetHeight)
                .andThen(Commands.waitUntil(() -> heightAtMost(SuperstructureConstants.LOW_BOX_ROOF)))
                .andThen(armTo(targetAngle))
                .andThen(settle());
        }
        // The arm leaves RAISE as the carriage passes the release height (or,
        // coming from above, drops into the window) and sweeps to the scoring
        // angle under the arm-side clamp, which holds it at the edge of any
        // row the carriage has not opened yet. It used to wait for a height
        // where the WHOLE sweep was clear at once.
        Command armReleased = Commands.waitUntil(
            () -> heightAtLeast(Math.min(SuperstructureConstants.ARM_RELEASE_MIN_HEIGHT,
                    targetHeight - SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET))
                && heightAtMost(SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT));

        // Mid scoring pose (L3): the carriage goes straight to the target at
        // RAISE and the arm sweeps during the last six inches of the climb,
        // instead of starting once the carriage had all but stopped. The
        // last row (25-30 deg) opens at 30.25 in, as the carriage arrives.
        if (targetHeight < SuperstructureConstants.L4_ZONE_MIN_HEIGHT) {
            return elevatorTo(targetHeight)
                .andThen(armReleased)
                .andThen(armWithCarriage(targetAngle, () -> Double.NaN))
                .andThen(settle());
        }
        // High scoring pose (L4): the arm comes down while the carriage
        // climbs behind it, its ceiling rising as the arm passes each row
        // (36 in at 95 deg, 40 at 70, 48 at 40, 51 at 25). Below the
        // final-angle height the arm is held at the stage angle - the one
        // robot-proven gate on the arm side that the table does not imply.
        boolean[] released = {false};
        Command armWork = armReleased
            .andThen(Commands.runOnce(() -> released[0] = true))
            .andThen(armWithCarriage(targetAngle,
                () -> heightAtLeast(SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT)
                    ? Double.NaN : SuperstructureConstants.L4_STAGE_ANGLE));
        // The last few inches still wait for the arm to REACH the scoring
        // angle. The corridor table says 25 deg is clear to 51 in, but the
        // robot says otherwise up there - L4 caught the top bar when the
        // carriage was allowed to climb past the pre-top height with the arm
        // still at 25 - so the pre-top height stays a hard ceiling until the
        // arm is inside its final gate, exactly as the staged version had it.
        // Everything below that is continuous.
        //
        // The carriage is no longer sent to the 33 in station and re-released
        // from there (a stop, then a retarget that went BACKWARDS to the
        // ratchet's first clamp). One ratchet owns it from the first loop:
        // it climbs straight to the clamp the arm's sweep allows (35.5 in
        // while the arm is at RAISE), the arm is released as the carriage
        // passes the release height, and the clamp opens as the arm comes
        // down - the earlier the release, the less the carriage has to slow
        // for that first clamp. Only a
        // carriage that starts ABOVE the rotation window comes down to the
        // station first.
        return Commands.deadline(armWork,
                travelWithArm(() -> {
                    if (!released[0]) {
                        // Arm still parked at RAISE: head for the release
                        // window - its top from below, the station from above.
                        return elevator.getCurrentPosition() > SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT
                            ? SuperstructureConstants.L4_STATION_HEIGHT
                            : SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT - SuperstructureConstants.RATCHET_MARGIN;
                    }
                    return armStrictlyAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)
                        ? targetHeight : SuperstructureConstants.L4_PRE_TOP_HEIGHT;
                },
                // Until the release the arm is PARKED at RAISE, not sweeping.
                // Clamping the carriage for a sweep that has not started is
                // wrong below 17.5 in, where every row's LOW band holds the
                // carriage and the sweep's ceiling is the low-box roof: a
                // carriage that was still down there when the arm reached
                // RAISE (a slow elevator, or L2 -> L4) stopped at 17 in while
                // the arm waited for it to reach the release height. Found by
                // SuperstructureClampSimTest.
                () -> released[0] ? targetAngle : SAFE_ANGLE))
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
            // The staged exits move the carriage (L4 drops to its station);
            // at RAISE every height is clear, so put it back where it was.
            double stay = elevator.getCurrentPosition();
            return escapeToSafe(stay)
                .andThen(both(stay, SAFE_ANGLE))
                .andThen(armArrived());
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

    /** Moves to the L3 scoring pose (30.5 in, 25 degrees): climb at RAISE, rotate within half an inch of the target. */
    public Command goToCoralL3() {
        return setGoal(Goal.CORAL_L3)
            .andThen(moveTo(PresetHeights.CORAL_L3, PivotPresetAngles.CORAL_L3));
    }

    /** Moves to the L4 scoring pose (52.5 in, 20 degrees) through the 33 in rotation station. */
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
        // Deliberately requires NO subsystem: the rollers share the CorAl
        // subsystem with the pivot, so requiring it made X cancel a move in
        // progress (leaving the carriage parked at whatever clamp the
        // ratchet last sent) and made any pose button cancel the eject.
        return Commands.sequence(
            Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.CORAL_SCORE_SPEED)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(coral::stopIntake)
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
                        + "All of 25-100 deg is clear at %.0f in; the low box (below %.0f in) allows %.0f deg and up.",
                        target, elevator.getCurrentPosition(), SuperstructureConstants.L4_STATION_HEIGHT,
                        SuperstructureConstants.LOW_BOX_ROOF, SuperstructureConstants.ARM_CLEAR_MIN_ANGLE)));
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
