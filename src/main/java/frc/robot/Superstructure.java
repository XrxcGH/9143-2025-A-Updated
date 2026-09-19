package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.CorAlConstants;
import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ArmAxis;
import frc.robot.subsystems.CarriageAxis;
import frc.robot.subsystems.Pace;
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
 * region in state-gated steps: every handover is triggered by a measured
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
 *   - Mid scoring pose (L3): the carriage climbs with the arm at RAISE;
 *     the arm is released as the carriage passes ARM_RELEASE_MIN_HEIGHT
 *     and sweeps to the scoring angle during the rest of the climb.
 *     Leaving: the carriage lifts to MID_POSE_RETURN_LIFT_HEIGHT while the
 *     arm swings up, and descends only once the arm has passed
 *     BAND_PASS_MIN_ANGLE (climbs on only once it is at RAISE) - descending
 *     any earlier sweeps the claw into the middle-stage top tube.
 *   - High scoring pose (L4): the top of travel is clear only with the arm
 *     at 22.5 deg or less, and the RAISE -> L4 rotation is impossible up
 *     there. The arm is released at the same height as for L3 and comes
 *     down while the carriage climbs behind it; it is held at
 *     L4_STAGE_ANGLE until the carriage is above L4_FINAL_ANGLE_MIN_HEIGHT,
 *     and the carriage is held at L4_PRE_TOP_HEIGHT until the arm is inside
 *     L4_FINAL_GATE_ANGLE. Leaving L4 is the mirror: the carriage drops
 *     first at the L4 angle, the arm starts back toward RAISE once the
 *     carriage is below L4_RETURN_ROTATE_MAX_HEIGHT, and the carriage
 *     waits at L4_RETURN_DROP_HEIGHT and then at L4_STATION_HEIGHT until
 *     the arm has opened the way below each.
 *   - L3 <-> L4: both poses lie in the same upper corridor, so the arm
 *     turns the few degrees between them at a height that suits both,
 *     with no excursion to RAISE (see {@link #directTransferHeight}).
 *   - Angles beyond HIGH_ANGLE_STAGE (algae poses) hit the bumper near the
 *     base, so the arm waits at the stage angle until the carriage is above
 *     HIGH_ANGLE_MIN_HEIGHT.
 *
 * How the moves are made continuous rather than stop-start:
 *   - Two clamps, one table. Every loop the carriage is given its final
 *     height clamped to what the corridors allow for the angles the arm
 *     still has to sweep ({@link #ceilingForSweep} / {@link #floorForSweep}),
 *     and the arm is given its final angle clamped to the rows the
 *     carriage's height has opened ({@link #armLimitForHeight}). Each
 *     mechanism is limited by the other's measured position, so the pair
 *     stays inside the corridors at any relative speed, including one of
 *     them stalling.
 *   - A setpoint latch (commandCarriage) sends the carriage a new setpoint
 *     only when its clamp has actually moved, because every new setpoint
 *     restarts the MAXMotion profile.
 *   - A {@link Pace} per move runs the carriage no faster than the arm can
 *     open the way, so the two travel together and arrive together.
 *
 * Reef safety: a move into or out of L3 / L4 swings the claw past the front
 * bumper, so it waits while a reef face is right in front of the robot
 * (see {@link #setNearReefSupplier}), and a finished L3 / L4 score waits
 * for the drivetrain to back off before it homes (see {@link #score}).
 *
 * The operator's manual take-over ({@link #manualOverride}) bypasses
 * these interlocks - the sticks command the subsystems directly and rely on
 * the operator watching the mechanism.
 */
public class Superstructure {

    /**
     * The pose family most recently commanded. Consumed by the vision
     * tracker to pick the right AprilTag alignment goal (e.g., at the reef:
     * a standoff for L1, flush on a branch for L2, and for L3 / L4 a
     * standoff until the pose is reached, then flush).
     */
    public enum Goal {
        STOW,
        CORAL_L1,
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        ALGAE_LOW,
        ALGAE_HIGH,
        ALGAE_SCORE,
        /** Carrying an algae low at the travel angle (the "home" pose while one is held). */
        ALGAE_CARRY
    }

    // Short names for two Constants values (no number of their own).
    /** Safe travel angle in degrees (RAISE preset): clear at every height. */
    private static final double SAFE_ANGLE = PivotPresetAngles.RAISE.getAngle();
    private static final double TOL = SuperstructureConstants.SAFE_ANGLE_TOLERANCE;

    private final CarriageAxis elevator;
    private final ArmAxis coral;

    // ------------------------------------------------------------------
    // Paces: how fast the carriage runs each kind of move (see Pace), from
    // SuperstructureConstants.*_PACE, where the choice of each is explained.
    // Package-private and not final only so that
    // SuperstructureSequenceSimTest.pacesCompared can vary them.
    // ------------------------------------------------------------------
    static Pace climbToL4Pace = SuperstructureConstants.CLIMB_TO_L4_PACE;
    static Pace climbToL3Pace = SuperstructureConstants.CLIMB_TO_L3_PACE;
    static Pace l4ExitPace = SuperstructureConstants.L4_EXIT_PACE;
    static Pace algaeToL4Pace = SuperstructureConstants.ALGAE_TO_L4_PACE;
    /** L4 exit: full pace down to here (just under the arm's 43 in release gate), paced from here on. */
    static double l4ExitShapeHeight = SuperstructureConstants.L4_RETURN_SHAPE_HEIGHT;
    /** How long before the carriage reaches the release height the arm is let go (see approach). */
    static double armReleaseLeadSeconds = SuperstructureConstants.ARM_RELEASE_LEAD_SECONDS;

    /** Last commanded pose family (drives vision alignment goals). */
    private Goal currentGoal = Goal.STOW;
    /** True from an algae intake / hold until it is scored or the rollers are run by hand: picks what "home" means. */
    private boolean algaeHeld = false;
    /** "A reef face is right in front of the bumper", raw (wired to vision in RobotContainer; false = never). */
    private BooleanSupplier nearReef = () -> false;
    /** FPGA time the raw reading was last true: "clear" has to persist, so a dropped camera frame does not release a swing. */
    private double lastNearReefTime = Double.NEGATIVE_INFINITY;
    /** True while a move is being held until the robot is clear of the reef. */
    private boolean waitingForReefClearance = false;

    public Superstructure(CarriageAxis elevator, ArmAxis coral) {
        this.elevator = elevator;
        this.coral = coral;
    }

    /**
     * Wires in "a reef face is right in front of the bumper". Every move into
     * or out of L3 / L4 swings the claw through 75-100 deg, where it reaches
     * 9-12 in past the front bumper (CAD) at the height of the reef's
     * branches - so with the robot against the reef that swing goes through
     * them. While this reads true such a move waits, holding both mechanisms
     * where they are, and starts by itself once the robot has backed away.
     * L3 <-> L4 (the arm stays at 20-25 deg), the low poses and the algae
     * poses (which reach into the reef by design) are not held. Manual
     * take-over (operator LB) is never held.
     */
    public void setNearReefSupplier(BooleanSupplier nearReef) {
        this.nearReef = nearReef;
    }

    /** "A reef face is right in front of the bumper" as the moves see it. Read every loop by the Dashboard, which also keeps its debounce fed. */
    public boolean isNearReef() {
        double now = Timer.getFPGATimestamp();
        if (nearReef.getAsBoolean()) {
            lastNearReefTime = now;
        }
        return now - lastNearReefTime < SuperstructureConstants.NEAR_REEF_RELEASE_SECONDS;
    }

    /** True while a move is being held until the robot has backed away from the reef (drives the driver's rumble). */
    public boolean isWaitingForReefClearance() {
        return waitingForReefClearance;
    }

    private Command waitForReefClearance() {
        return Commands.waitUntil(() -> !isNearReef())
            .beforeStarting(() -> waitingForReefClearance = isNearReef())
            .finallyDo(() -> waitingForReefClearance = false);
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

    /** True if every height from {@code from} to {@code to} (PATH_CHECK_HEIGHT_STEP steps) is clear at {@code angle}. */
    public static boolean elevatorPathClear(double from, double to, double angle) {
        double lo = Math.min(from, to), hi = Math.max(from, to);
        for (double h = lo; h <= hi + 1e-9; h += SuperstructureConstants.PATH_CHECK_HEIGHT_STEP) {
            if (!poseClear(Math.min(h, hi), angle)) {
                return false;
            }
        }
        return poseClear(hi, angle);
    }

    /** True if every angle from {@code from} to {@code to} (PATH_CHECK_ANGLE_STEP steps) is clear at {@code height}. */
    public static boolean pivotPathClear(double from, double to, double height) {
        double lo = Math.min(from, to), hi = Math.max(from, to);
        for (double a = lo; a <= hi + 1e-9; a += SuperstructureConstants.PATH_CHECK_ANGLE_STEP) {
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

    /** One-line description of the L3 / L4 handoff gates, from the live constants, for the dashboard. */
    public static String stationSummary() {
        return String.format(
            "L3 / L4 up: arm leaves RAISE as the carriage passes %.0f in | "
                + "L4 up: arm waits at %.1f deg below %.0f in, carriage waits at %.0f in until the arm is inside %.1f deg | "
                + "L4 down: arm to RAISE below %.0f in; carriage holds %.0f in until %.0f deg, %.0f in until %.0f deg | "
                + "L3 down: lift to %.0f in, descend past %.0f deg | "
                + "L3 <-> L4: direct, arm turns inside the band overlap (no RAISE)",
            SuperstructureConstants.ARM_RELEASE_MIN_HEIGHT,
            SuperstructureConstants.L4_STAGE_ANGLE, SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT,
            SuperstructureConstants.L4_PRE_TOP_HEIGHT, SuperstructureConstants.L4_FINAL_GATE_ANGLE,
            SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT,
            SuperstructureConstants.L4_RETURN_DROP_HEIGHT, SuperstructureConstants.L4_RETURN_STAGE_DONE_ANGLE,
            SuperstructureConstants.L4_STATION_HEIGHT, SuperstructureConstants.BAND_PASS_MIN_ANGLE,
            SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT, SuperstructureConstants.BAND_PASS_MIN_ANGLE);
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
     * Strict form of {@link #armAtMost}, for a gate that is a clearance
     * limit rather than a "close enough to carry on" check. The tolerant
     * form accepts SAFE_ANGLE_TOLERANCE past the gate, which for the L4
     * pre-top gate (22.5 deg) would mean it opens at 25.5 - an angle at
     * which the claw meets the top bar above the pre-top height. The arm's
     * L4 destination is 20 deg, so it passes 22.5 on the way and this
     * cannot stall.
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

    private Command elevatorTo(double height, Pace pace) {
        return Commands.runOnce(() -> elevator.setPosition(height, pace), elevator);
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
     * that stalls cannot deadlock the command (and with it whatever is
     * sequenced after it - the roller start, an auto's next step). Ending
     * on "stopped" is what lets the
     * tolerances be tight: once motion has ceased the closed loops are
     * holding their latched setpoints and waiting longer cannot improve the
     * pose. A short minimum dwell keeps "stopped" from firing before the
     * mechanisms have started. Intermediate safety gates deliberately do
     * not get this treatment.
     */
    private Command settle() {
        // One debouncer per settle (plans are built fresh by Commands.defer):
        // a velocity reading passing through zero must not count as "stopped".
        Debouncer stopped = new Debouncer(SuperstructureConstants.SETTLE_STOPPED_DEBOUNCE_SECONDS);
        return Commands.waitSeconds(SuperstructureConstants.SETTLE_MIN_SECONDS)
            .andThen(Commands.waitUntil(() -> atTargets() || stopped.calculate(bothStopped())))
            .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS);
    }

    /**
     * Highest carriage height that is clear with the arm at the given
     * measured angle while it swings up toward RAISE (used for the climbing
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
    // The sequences below are safe because every handover waits on
    // measured state. Parking the carriage at each intermediate height
    // while the arm works would make every move a stop-start routine. These
    // helpers avoid those stops without touching the handover logic: the
    // carriage is given the final height every loop, clamped to what the
    // corridors allow for the angles the arm still has to sweep, so it
    // climbs (or descends) continuously as the arm unlocks travel.
    //
    // This is cheap - one walk of the 32-row corridor table per loop, no
    // search - and it cannot deadlock, because the arm's schedule is
    // unchanged and its direction within a stage is known: the limit only
    // moves in the carriage's favour as the arm advances.

    // "No band here" marker (NaN), not a setting
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
     *
     * Fails closed. The walk stops at the first angle whose band does not
     * contain this height. That is legitimate when moving on is what
     * unlocks that angle - its nearest band is ahead of the carriage (the
     * L4 climb: 20 deg only opens above 35.5 in, ahead of a carriage that
     * is still climbing toward it). It is not legitimate when the arm is
     * about to enter that angle (it is within IMMINENT_SWEEP of where the
     * arm is now) and the row's nearest band is behind the carriage - it
     * overshot that row's ceiling - or when the current pose is outside
     * the table altogether. Both of those return the current height (hold):
     * "no limit" there would release the carriage exactly when it should
     * hold. A blocked row far along the sweep is the end of the walk:
     * the arm is gated on the carriage's height before it gets there.
     */
    public static double ceilingForSweep(double from, double to, double height) {
        double ceiling = sweepCeiling(from, to, height);
        return Double.isNaN(ceiling) ? height : ceiling; // hold: do not climb into it
    }

    /** {@link #ceilingForSweep}, with NaN for "hold where you are" so the caller can tell it from a real ceiling. */
    private static double sweepCeiling(double from, double to, double height) {
        double ceiling = ElevatorConstants.ELEVATOR_MAX_POSITION;
        double start = snapIntoTable(from, height);
        double step = to >= start ? SuperstructureConstants.SWEEP_STEP : -SuperstructureConstants.SWEEP_STEP;
        double angle = start;
        boolean first = true;
        while (true) {
            double rowCeiling = bandCeiling(angle, height);
            if (first) {
                rowCeiling = tighter(rowCeiling, bandCeiling(angle - SuperstructureConstants.EDGE_WINDOW, height), true);
                rowCeiling = tighter(rowCeiling, bandCeiling(angle + SuperstructureConstants.EDGE_WINDOW, height), true);
            }
            if (Double.isNaN(rowCeiling)) {
                if (first || (Math.abs(angle - start) <= SuperstructureConstants.IMMINENT_SWEEP
                        && !rowOpensToward(angle, height, true))) {
                    return Double.NaN;
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
        double floor = sweepFloor(from, to, height);
        return Double.isNaN(floor) ? height : floor; // hold: do not descend into it
    }

    private static double sweepFloor(double from, double to, double height) {
        double floor = ElevatorConstants.ELEVATOR_ZERO_HEIGHT;
        double start = snapIntoTable(from, height);
        double step = to >= start ? SuperstructureConstants.SWEEP_STEP : -SuperstructureConstants.SWEEP_STEP;
        double angle = start;
        boolean first = true;
        while (true) {
            double rowFloor = bandFloor(angle, height);
            if (first) {
                rowFloor = tighter(rowFloor, bandFloor(angle - SuperstructureConstants.EDGE_WINDOW, height), false);
                rowFloor = tighter(rowFloor, bandFloor(angle + SuperstructureConstants.EDGE_WINDOW, height), false);
            }
            if (Double.isNaN(rowFloor)) {
                if (first || (Math.abs(angle - start) <= SuperstructureConstants.IMMINENT_SWEEP
                        && !rowOpensToward(angle, height, false))) {
                    return Double.NaN;
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

    /** The tighter of two limits (min of ceilings / max of floors); a NaN side - that row does not hold this height - is ignored. */
    private static double tighter(double a, double b, boolean ceilings) {
        if (Double.isNaN(a)) {
            return b;
        }
        if (Double.isNaN(b)) {
            return a;
        }
        return ceilings ? Math.min(a, b) : Math.max(a, b);
    }

    /**
     * The measured arm angle, nudged by at most SAFE_ANGLE_TOLERANCE onto a
     * table row that contains this height. A scoring pose that sits ON a
     * row boundary (L3 is 25.0 deg, the edge of the 25-30 row whose
     * corridor starts at 30 in) reads 24.x on the through bore once the
     * chain settles, and the 20-25 row says 30.5 in is blocked - so without
     * the nudge every lookup from the resting L3 pose would fail, and the
     * L3 -> L4 direct transfer would never be taken. The exposure is the one
     * the resting pose already has.
     */
    static double snapIntoTable(double angle, double height) {
        if (!Double.isNaN(bandCeiling(angle, height))) {
            return angle;
        }
        for (double d = SuperstructureConstants.SNAP_ANGLE_STEP; d <= SuperstructureConstants.SAFE_ANGLE_TOLERANCE + 1e-9;
                d += SuperstructureConstants.SNAP_ANGLE_STEP) {
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
     * True if the band of this angle's row nearest to {@code height} lies on
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
    // MAXMotion restarts its profile from the measured position and a
    // lagged measured velocity every time the setpoint changes. A ratchet
    // that sent max(clamp, measuredHeight) every loop would change the
    // setpoint every 20 ms at a clamp and restart the profile fifty times a
    // second, which shows up as the carriage hunting. The latch sends a new
    // setpoint only when the clamp has actually moved, and holds off small
    // steps while the carriage is still far from needing them. Its steps and
    // windows are SuperstructureConstants.LATCH_*, DIRECTION_EPSILON and
    // RAISE_ARM_PUT_BACK_MIN.

    /** Where the carriage comes to rest if told to stop now (no overshoot-and-return, unlike "hold the measured height"). */
    private double stoppingPoint() {
        return stoppingPoint(Pace.FULL);
    }

    /** ...at the acceleration of the pace the stopping setpoint will be sent with (not the one in force now). */
    private double stoppingPoint(Pace pace) {
        double v = elevator.getVelocity();
        return elevator.getCurrentPosition()
            + Math.copySign(v * v / (2.0 * Math.max(elevator.maxAcceleration(pace), 1.0)), v);
    }

    /** The clamp, or - when the carriage is already past it - the clamp if it is close, else where the carriage can stop. */
    private double withinReach(double desired, boolean up, Pace pace) {
        double height = elevator.getCurrentPosition();
        double past = up ? height - desired : desired - height;
        if (past <= 0 || past <= SuperstructureConstants.LATCH_RETREAT_MAX) {
            return desired;
        }
        double stop = stoppingPoint(pace);
        return up ? Math.max(desired, stop) : Math.min(desired, stop);
    }

    /**
     * Sends {@code desired} to the carriage through a per-command latch
     * ({@code latch[0]}, NaN until first use). {@code up} is the travel
     * direction the desired value was computed for; a NaN {@code desired}
     * means the table has no answer here: stop, and wait for the arm.
     */
    private void commandCarriage(double[] latch, double desired, double finalTarget, boolean up) {
        commandCarriage(latch, desired, finalTarget, up, Pace.FULL);
    }

    private Pace lastCarriagePace = Pace.FULL;

    private void commandCarriage(double[] latch, double desired, double finalTarget, boolean up, Pace pace) {
        if (Double.isNaN(desired)) {
            if (Double.isNaN(latch[0])) {
                latch[0] = stoppingPoint(pace);
                elevator.setPosition(latch[0], pace);
            }
            return;
        }
        if (Double.isNaN(latch[0])) {
            // A carriage at rest does not set off on a short run to a clamp:
            // it would accelerate, brake for the clamp, and be re-released
            // when the arm opens it - a hitch, for nothing, because the arm is
            // the long pole and the clamp will have opened by the time a
            // proper run is available. (Base -> anything: the 8.5 in tuck
            // limit; L2 -> L3 / L4: the 16.5 in roof, 4.5 in away.) The arm
            // is moving meanwhile, so nothing stands still.
            double run = up ? desired - elevator.getCurrentPosition() : elevator.getCurrentPosition() - desired;
            boolean clamped = up ? desired < finalTarget - 1e-6 : desired > finalTarget + 1e-6;
            if (clamped && run >= 0 && run < SuperstructureConstants.LATCH_MIN_RUN
                    && Math.abs(elevator.getVelocity()) <= SuperstructureConstants.LATCH_CRAWL_SPEED) {
                return;
            }
            latch[0] = withinReach(desired, up, pace);
            lastCarriagePace = pace;
            elevator.setPosition(latch[0], pace);
            return;
        }
        double advance = up ? desired - latch[0] : latch[0] - desired;
        if (advance > 1e-6) {
            boolean reachesTarget = up ? desired >= finalTarget - 1e-6 : desired <= finalTarget + 1e-6;
            boolean crawling = Math.abs(elevator.getVelocity()) <= SuperstructureConstants.LATCH_CRAWL_SPEED
                && advance >= SuperstructureConstants.LATCH_CRAWL_MIN_STEP;
            if (reachesTarget || advance >= SuperstructureConstants.LATCH_MIN_STEP || crawling || pace != lastCarriagePace) {
                latch[0] = desired;
                lastCarriagePace = pace;
                elevator.setPosition(desired, pace);
            }
        } else if (advance < -SuperstructureConstants.LATCH_RETREAT_MIN) {
            // The clamp moved back (a gate re-closed, or the carriage arrived
            // past it). One new setpoint, not a chase of the measured height.
            double backTo = withinReach(desired, up, pace);
            if (Math.abs(backTo - latch[0]) > SuperstructureConstants.LATCH_RETREAT_MIN) {
                latch[0] = backTo;
                elevator.setPosition(backTo, pace);
            }
        }
    }

    /**
     * The arm angle the carriage's clamp is walked from: the measured angle,
     * until the arm has come within its at-target window of where it is
     * going - from then on, the destination itself, for good. A pose on a row
     * edge is otherwise never "reached" as far as the table is concerned: the
     * reading hovers either side of 25.0 deg and the carriage's floor flips
     * between 30 and 35.5 in with it (on L4 -> L3 the carriage would be sent
     * 36, 30.5, 36, 30.5, ...). The exposure is the one the resting pose
     * already has. {@code arrival} is
     * per-command state: {destination it arrived at (NaN = not yet), unused}.
     */
    private double armAngleForSweep(double[] arrival, double destination) {
        double measured = coral.getPivotAngle();
        if (arrival[0] != destination) {
            arrival[0] = Double.NaN; // a new destination: not there yet
        }
        if (Math.abs(measured - destination) <= CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR) {
            arrival[0] = destination;
        }
        return Double.isNaN(arrival[0]) ? measured : destination;
    }

    /** The height a climb toward {@code target} may be commanded to right now. */
    private double climbGoal(double target, double armAngle, double armDestination) {
        double raw = sweepCeiling(armAngle, armDestination, elevator.getCurrentPosition());
        if (Double.isNaN(raw)) {
            return Double.NaN; // hold
        }
        // A target the corridor allows is commanded as it is - the scoring
        // poses sit near the top of their band by design, so subtracting a
        // margin from those would leave the carriage permanently low. Only
        // a target the corridor does not allow is clamped, and then it stops
        // a margin short of the limit rather than exactly on it, so the
        // tracking error of a carriage decelerating into its clamp stays
        // inside the corridor.
        return target <= raw ? target : raw - SuperstructureConstants.RATCHET_MARGIN;
    }

    /** Mirror of {@link #climbGoal}: a legal target as it is, an illegal one a margin above the floor. */
    private double descentGoal(double target, double armAngle, double armDestination) {
        double raw = sweepFloor(armAngle, armDestination, elevator.getCurrentPosition());
        if (Double.isNaN(raw)) {
            return Double.NaN; // hold
        }
        return target >= raw ? target : raw + SuperstructureConstants.RATCHET_MARGIN;
    }

    /**
     * Travels toward {@code target} in whichever direction it lies, as fast
     * as the arm's sweep allows - the climb ratchet when the target is
     * above, the descent ratchet when it is below. Used wherever the arm has
     * work to do and the carriage would otherwise sit still waiting for it.
     */
    private Command travelWithArm(DoubleSupplier targetSupplier, DoubleSupplier armDestination) {
        return travelWithArm(targetSupplier, armDestination, () -> Pace.FULL);
    }

    private Command travelWithArm(DoubleSupplier targetSupplier, DoubleSupplier armDestination, Supplier<Pace> pace) {
        double[] latch = {Double.NaN};
        boolean[] wasUp = {true};
        double[] arrival = {Double.NaN, 0.0};
        return Commands.run(() -> {
            double target = targetSupplier.getAsDouble();
            // Direction from the last commanded height once there is one, so
            // a carriage resting a hair either side of its setpoint does not
            // flip the ratchet back and forth. A target equal to what is
            // already commanded keeps the direction it had: a plain
            // "target >= reference" would read a finished descent (latch ==
            // target) as a climb, and the ratchet would flip direction every
            // loop - alternating "go to 39 in" with "hold where you are"
            // every 20 ms, which shakes the elevator on the L4 exit and
            // keeps it from getting down.
            double reference = Double.isNaN(latch[0]) ? elevator.getCurrentPosition() : latch[0];
            double epsilon = SuperstructureConstants.DIRECTION_EPSILON;
            boolean up = target > reference + epsilon ? true
                : target < reference - epsilon ? false
                : wasUp[0];
            if (Double.isNaN(latch[0]) && Math.abs(target - reference) <= epsilon) {
                up = target >= elevator.getCurrentPosition(); // nothing commanded yet and already there
            }
            if (up != wasUp[0]) {
                latch[0] = Double.NaN; // the target moved to the other side: start a fresh latch
                wasUp[0] = up;
            }
            double destination = armDestination.getAsDouble();
            double armAngle = armAngleForSweep(arrival, destination);
            double goal = up ? climbGoal(target, armAngle, destination)
                : descentGoal(target, armAngle, destination);
            commandCarriage(latch, goal, target, up, pace.get());
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
     * Two poses whose corridor bands overlap do not need the RAISE
     * excursion. L3 (25 deg, clear 30-51 in) and L4 (20 deg, clear
     * 35.5-52.5 in) overlap over 35.5-51 in, so anywhere in there the arm
     * turns the five degrees between them; the general plan would
     * rotate 75 deg out to RAISE and 80 back to avoid those five.
     *
     * Checked against every ordered pair of operator poses, this is the only
     * pair it changes, and that is geometry rather than luck:
     *   - BASE/L2 to L3/L4 - the low box (up to ~20 in) and the upper
     *     corridor (from ~30 in) do not overlap at any low angle, because
     *     the middle-stage top tube sits between them. The arm has to come
     *     up to 75-100 deg to cross, so the excursion is the move.
     *   - anything to L1, the algae poses, or from them - those poses are at
     *     or beyond RAISE already, so there is no excursion to remove; the
     *     general plan commands both mechanisms together.
     * That leaves L3 and L4, the two poses that share the upper corridor.
     *
     * The turn happens at the point of the overlap nearest the carriage, so
     * it starts as soon as the carriage gets there and never detours to
     * reach it. It is never above the pre-top height: the table is
     * optimistic at the top of travel (at 25 deg above 48 in the claw meets
     * the top bar on the real robot) and the pre-top height is the ceiling
     * the robot has actually held the carriage at while the arm swept
     * through these angles.
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
        // The measured start angle, nudged onto the row that holds this
        // height: a resting L3 reads 24.x deg, a hair under its own row.
        fromAngle = snapIntoTable(fromAngle, fromHeight);
        double fromLo = bandFloor(fromAngle, fromHeight);
        double toLo = bandFloor(toAngle, target);
        if (Double.isNaN(fromLo) || Double.isNaN(toLo)) {
            return NO_BAND; // one of the poses is not in a corridor at all
        }
        // Both poses in the upper corridor, which is what this is for. The
        // 70-75 deg row is one band from the base to 39 in, so a second
        // button press that catches the arm at 72 deg on its way down would
        // otherwise "overlap" the base pose: the transfer would be chosen
        // with a turn height of 8 in, the arm (not re-commanded until the
        // turn) would carry on to its old target, and the carriage, clamped
        // above the floor that puts in its way, would never get down to the
        // turn - each waiting for the other. Covered by
        // SuperstructureSequenceSimTest.secondButtonMidMove.
        if (fromLo <= SuperstructureConstants.LOW_BOX_ROOF || toLo <= SuperstructureConstants.LOW_BOX_ROOF) {
            return NO_BAND;
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
     * the free corridors allow. Deferred so the plan is built from the real
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
        // "Starts low" means it will still be low once it has stopped: a
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
                // A tucked arm above the tuck limit is a manual-control-only
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
            // to clear ~8 deg before the carriage may leave the base),
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
            double lowerTo = Math.max(target, SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT);
            if (targetAngle < SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE) {
                // Heading for a scoring pose: no higher than the top of the
                // window the arm is released in. Sending the carriage to the
                // final height (51.5 in for L4) while the arm comes back to
                // RAISE would only have the approach bring it all the way
                // back down into that window: a yo-yo.
                lowerTo = Math.min(lowerTo,
                    SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT - SuperstructureConstants.RATCHET_MARGIN);
            }
            lowerFirst = both(lowerTo, holdAngle)
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
        // Into or out of an upper scoring pose: the arm swings out past the
        // front bumper on the way, so not while a reef face is right there.
        boolean leavesUpperPose = h0 > SuperstructureConstants.LOW_BOX_ROOF
            && a0 < SuperstructureConstants.BAND_PASS_MIN_ANGLE;
        boolean entersUpperPose = target > SuperstructureConstants.LOW_BOX_ROOF
            && targetAngle < SuperstructureConstants.BAND_PASS_MIN_ANGLE;
        Command clearOfReef = leavesUpperPose || entersUpperPose ? waitForReefClearance() : Commands.none();
        // From an algae pose the arm has 140 deg to come round before L4 (it
        // is even more the long pole), so the carriage takes it easier still.
        Pace climbPace = !entersUpperPose ? Pace.FULL
            : target < SuperstructureConstants.L4_ZONE_MIN_HEIGHT ? climbToL3Pace
            : a0 > SuperstructureConstants.HIGH_ANGLE_STAGE ? algaeToL4Pace : climbToL4Pace;
        return clearOfReef.andThen(lowerFirst).andThen(escapeToSafe(target, climbPace))
            .andThen(approach(target, targetAngle, climbPace));
    }

    /**
     * Runs a direct scoring-pose transfer: travel to the turn height, and
     * turn there while the carriage carries on toward the target behind the
     * ratchet. The carriage is moving the whole time, and above the pre-top
     * height it still waits for the arm's final gate - the one limit the
     * robot, rather than the table, put there.
     *
     * The last leg is commanded once the arm has arrived, because a pose
     * whose band floor is a row boundary (L3 at exactly 25 deg) is only
     * reachable with the arm at its angle; the ratchet alone would hold the
     * carriage at the previous row's floor.
     */
    private Command directTransfer(double startHeight, double rotateAt, double target, double targetAngle) {
        // The turn starts when the carriage reaches the turn height from the
        // side it started on - not within a window around it. The turn
        // height sits inside the overlap of the two bands with the ratchet
        // margin to spare, but a window around it does not: a symmetric one
        // would let the arm start turning an inch short of the overlap,
        // which for the L3 climb is below the 20-25 deg corridor floor.
        //
        // The carriage is NEVER sent to the turn height. If it were, the
        // gate would wait for the carriage to reach the very height its
        // profile is stopping at: a carriage resting 0.02 in short would
        // never open it (the gate has no timeout), and even when it did
        // open the carriage would have stopped first. Instead the carriage
        // travels toward the real target behind the ratchet from the first
        // loop, and the turn height is only the threshold the arm waits for
        // as the carriage goes past.
        Command reachTurn = rotateAt >= startHeight
            ? Commands.waitUntil(() -> heightAtLeast(rotateAt))
            : Commands.waitUntil(() -> heightAtMost(rotateAt));
        boolean gated = target > SuperstructureConstants.L4_PRE_TOP_HEIGHT;
        DoubleSupplier carriageTarget = () ->
            gated && !armStrictlyAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)
                ? SuperstructureConstants.L4_PRE_TOP_HEIGHT : target;
        // The arm turns under the arm-side clamp like every other sweep,
        // rather than being sent straight to the target once the gate opens:
        // a second press that catches the carriage just under 48 in on its
        // way up opens the gate at once, and an unclamped arm would leave
        // the L4 angle while the carriage coasts on to 50 in. Covered by
        // SuperstructureSequenceSimTest.secondButtonMidMove.
        Command armWork = reachTurn
            .andThen(armWithCarriage(targetAngle, () -> Double.NaN));
        return Commands.deadline(armWork, travelWithArm(carriageTarget, () -> targetAngle))
            // Last leg once the arm has arrived (see above) - but never past
            // the pre-top height unless the arm is actually inside its gate.
            .andThen(Commands.either(elevatorTo(target), Commands.none(),
                () -> !gated || armStrictlyAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)))
            .andThen(settle());
    }

    // ==================================================================
    // Arm-side clamp ("the other half of the ratchet")
    // ==================================================================
    // The carriage is clamped by where the arm is (ceilingForSweep /
    // floorForSweep). An arm that was merely gated - parked at RAISE until
    // the carriage reached a window where its whole sweep was clear at once
    // (31-36 in) - would have the carriage arrive at its 35.5 in clamp with
    // the arm barely moving and stop there, and would make the L3 rotation
    // wait for the carriage to finish. So the arm is clamped by where the
    // carriage is - the mirror image, one walk of the same table per loop:
    // it can start as soon as its first rows are clear and is held at
    // the edge of any row the carriage has not opened yet. Each mechanism is
    // limited by the other's measured position, so the pair stays inside the
    // corridors at any relative speed, including one of them stalling.

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
    private static boolean rowHolds(double[] row, double height, double floorMargin, double ceilingMargin) {
        for (int b = 1; b <= 3; b += 2) {
            if (Double.isNaN(row[b])) {
                continue;
            }
            boolean aboveFloor = row[b] <= 0.0 ? height >= row[b] : height >= row[b] + floorMargin;
            if (aboveFloor && height <= row[b + 1] - ceilingMargin) {
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
        if (from < 0 || to < 0 || !rowHolds(rows[from], height, 0.0, 0.0)) {
            return angle;
        }
        // A rule measured on the robot, which the table does not contain:
        // above the pre-top height the arm stays inside the final gate angle
        // (the table says 25 deg is clear to 51 in; on the robot the claw
        // meets the top bar at 25 deg above 48 in). The carriage side
        // enforces it on the way up; an arm leaving L4 has to as well.
        double gate = SuperstructureConstants.L4_FINAL_GATE_ANGLE;
        boolean aboveGate = height > SuperstructureConstants.L4_PRE_TOP_HEIGHT
            && angle < SuperstructureConstants.BAND_PASS_MIN_ANGLE;
        if (aboveGate && angle > gate) {
            return angle; // already outside it up here: hold, the carriage has to come down
        }
        // (A margin inside the gate, like every other arm limit: the arm is
        // never sent to sit ON a boundary it must not cross.)
        double capped = aboveGate
            ? Math.min(target, gate - SuperstructureConstants.ARM_CLAMP_ANGLE_MARGIN) : target;
        int step = to < from ? -1 : 1;
        for (int j = from; j != to;) {
            j += step;
            // Sweeping up (leaving L4) the arm runs into the ceilings - the
            // claw's rear against the middle-stage top tube - which is where
            // the table has proved optimistic, so it keeps a real margin under
            // them. Sweeping down it runs away from them, and the margin must
            // stay inside the carriage's own (RATCHET_MARGIN) or the two
            // clamps would wait on each other.
            // (Rows above the L4 gate angle only: the L4 pose itself sits an
            // inch under its own row's ceiling by design.)
            boolean bandB = j > 0 && rows[j - 1][0] >= SuperstructureConstants.L4_FINAL_GATE_ANGLE;
            double ceilingMargin = step > 0 && bandB ? SuperstructureConstants.ARM_CLAMP_CEILING_MARGIN_RISING
                : SuperstructureConstants.ARM_CLAMP_HEIGHT_MARGIN;
            if (!rowHolds(rows[j], height, SuperstructureConstants.ARM_CLAMP_HEIGHT_MARGIN, ceilingMargin)) {
                // The near edge of the blocked row: its upper bound going
                // down, the previous row's upper bound going up.
                double limit = step < 0
                    ? rows[j][0] + SuperstructureConstants.ARM_CLAMP_ANGLE_MARGIN
                    : rows[j - 1][0] - SuperstructureConstants.ARM_CLAMP_ANGLE_MARGIN;
                return step > 0 ? Math.min(limit, capped) : limit;
            }
        }
        return capped;
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
        return armWithCarriage(targetAngle, extraLimit, null);
    }

    /** As above; {@code committed[0]}, when given, is kept at the limit the arm has actually been sent to. */
    private Command armWithCarriage(double targetAngle, DoubleSupplier extraLimit, double[] committed) {
        double[] finalSince = {Double.NaN};
        return Commands.run(() -> {
            double angle = coral.getPivotAngle();
            // Where the carriage is and where it comes to rest: a carriage
            // still coasting upward is about to be somewhere else, and the
            // arm must be legal there too. The tighter of the two.
            double here = armLimitForHeight(angle, targetAngle, elevator.getCurrentPosition());
            double v = elevator.getVelocity();
            double stopsAt = elevator.getCurrentPosition()
                + Math.copySign(v * v / (2.0 * Math.max(elevator.maxAcceleration(), 1.0)), v);
            double there = armLimitForHeight(angle, targetAngle, stopsAt);
            double limit = targetAngle < angle ? Math.max(here, there) : Math.min(here, there);
            double extra = extraLimit.getAsDouble();
            if (!Double.isNaN(extra)) {
                limit = targetAngle < angle ? Math.max(limit, extra) : Math.min(limit, extra);
            }
            coral.setPivotAngle(limit);
            if (committed != null) {
                committed[0] = limit;
            }
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
     * outside it for good - and a wait on the window alone would then park
     * both mechanisms for the whole 3 s timeout. Once the arm has stopped,
     * waiting longer cannot improve the angle.
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
     * Ends with the arm at/above SAFE_TRAVEL_MIN_ANGLE - or, when the next
     * leg is a descent out of L3 / L4, as soon as the arm has passed
     * BAND_PASS_MIN_ANGLE on its way there (from 75 deg up, everything below
     * is clear). Never times out: these are safety gates.
     */
    private Command escapeToSafe(double targetHeight) {
        return escapeToSafe(targetHeight, Pace.FULL);
    }

    private Command escapeToSafe(double targetHeight, Pace climbPace) {
        double h0 = elevator.getCurrentPosition();
        double a0 = coral.getPivotAngle();

        if (a0 >= SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE - TOL) {
            // Free to travel - IF the arm is staying here. A second button
            // press mid-move can find the arm at 99 deg on its way down to a
            // scoring angle (or on its way out to 160) with the interrupted
            // plan's setpoint still latched on the TalonFX: unless the arm
            // is re-commanded it carries on while the new plan drives the
            // carriage through a band that is only clear at RAISE. Re-latch it.
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
            // box (e.g., 90 deg at 38 in after manual control): rotating on
            // toward RAISE moves the claw's rear away from the tube. The
            // carriage travels toward the target underneath it rather than
            // waiting, clamped to what the arm's measured angle allows.
            return Commands.deadline(
                armTo(SAFE_ANGLE)
                    .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE))),
                travelWithArm(() -> targetHeight, () -> SAFE_ANGLE));
        }

        // In the low box. Swing to RAISE, and travel toward the target the
        // whole time: up as far as the arm's measured angle allows,
        // re-evaluated every loop so the ceiling ratchets up as the arm
        // swings (tuck limit -> low box roof -> mid corridor -> anything),
        // or down behind the falling floor. The setpoint latch keeps the
        // retargets few, so the carriage keeps moving smoothly. Either way
        // the carriage is moving while the arm is, so nothing waits.
        double[] climbLatch = {Double.NaN};
        return Commands.deadline(
            armTo(SAFE_ANGLE)
                .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE))),
            targetHeight > h0
                ? Commands.run(() -> {
                    commandCarriage(climbLatch,
                        Math.min(targetHeight, climbCeiling(coral.getPivotAngle())), targetHeight, true, climbPace);
                }, elevator)
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
        // Once band A is cleared the carriage heads for the real target, up
        // or down - never for the hard stop regardless, or a move up from L3
        // (and raiseArm / holdAlgae) would dive toward the base first.
        return Commands.deadline(armWork,
            travelWithArm(() -> armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE) ? targetHeight : lift,
                () -> SAFE_ANGLE));
    }

    /**
     * Leaves the high scoring pose (L4): the carriage drops first at the L4
     * angle; once it is below L4_RETURN_ROTATE_MAX_HEIGHT the arm heads for
     * RAISE under the arm-side clamp while the carriage keeps descending -
     * to the drop height, then the station, then (band A cleared) the
     * target.
     */
    private Command leaveHighPose(double targetHeight) {
        boolean descendingNext = targetHeight < SuperstructureConstants.L4_STATION_HEIGHT;
        double release = descendingNext ? SuperstructureConstants.BAND_PASS_MIN_ANGLE
            : SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE;
        // The carriage drops first (the top is only clear at the L4 angle),
        // and once it is low enough for the arm to start swinging back, the
        // two run together: the arm sweeps up to RAISE while the carriage
        // keeps descending, its target stepping down as the arm rises (the
        // drop height until the arm has passed 40 deg, the station until it
        // has passed 75, then the real target). One continuous drop instead
        // of drop-rotate-drop.
        // The arm heads for RAISE in one go, clamped every loop by the
        // carriage's height (45-50 deg is open at 43 in, 70 at 39, RAISE
        // below 35). Staging it at 45 deg first would put a velocity trough
        // in the middle of the swing, and a second leg sent on from there
        // would have no height gate at all.
        Command armWork = Commands.deadline(
            Commands.waitUntil(() -> armAtLeast(release)),
            armWithCarriage(SAFE_ANGLE, () -> Double.NaN));
        // Mirror of the climb: continuous down to the station, and below it
        // only once the arm has cleared band A.
        // One command owns the carriage for the whole exit, so there is no
        // loop in which it sits on a stale setpoint between steps. Below the
        // station it heads for the real target, not the hard stop (the next
        // pose may be above).
        Command gatedArmWork = Commands.waitUntil(
                () -> heightAtMost(SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT))
            .andThen(armWork);
        return Commands.deadline(gatedArmWork,
            travelWithArm(() -> {
                if (armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE)) {
                    // Band A cleared: on down to the target. A target above the
                    // station waits there until the arm is at RAISE (the approach
                    // takes it up): climbing toward a ceiling while the arm is still
                    // sweeping up under its own, larger, margin to the same ceiling
                    // would leave each waiting for the other (L4 -> high algae:
                    // stuck at 35.5 in / 87.5 deg).
                    return Math.min(targetHeight, SuperstructureConstants.L4_STATION_HEIGHT);
                }
                if (armAtLeast(SuperstructureConstants.L4_RETURN_STAGE_DONE_ANGLE)) {
                    return SuperstructureConstants.L4_STATION_HEIGHT;      // staged: the station (load-bearing)
                }
                // Arm still near the L4 angle. The first leg runs at full pace
                // to a height just under the arm's release gate, and only the
                // last stretch to the drop height is paced to the arm.
                return elevator.getCurrentPosition() > l4ExitShapeHeight + SuperstructureConstants.L4_RETURN_SHAPE_WINDOW
                    ? l4ExitShapeHeight : SuperstructureConstants.L4_RETURN_DROP_HEIGHT;
            }, () -> SAFE_ANGLE,
            // Paced while the arm is on its way round (so the carriage arrives
            // at the drop height and the station as the arm opens them, rather
            // than stopping at each); full pace once band A is cleared.
            () -> armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE)
                || elevator.getCurrentPosition() > l4ExitShapeHeight + SuperstructureConstants.L4_RETURN_SHAPE_WINDOW
                ? Pace.FULL : l4ExitPace));
    }

    /**
     * Travels to the target and settles at the target angle, given the arm
     * is at/above the safe travel angle. Picks the strategy by target type.
     */
    private Command approach(double targetHeight, double targetAngle, Pace climbPace) {
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
        // Free-zone target (RAISE .. stage angle, e.g., L1, algae score): together
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
        // row the carriage has not opened yet - it does not wait for a height
        // where the whole sweep is clear at once.
        // Released with a lead: the arm takes ~0.2 s to move its first few
        // degrees, so it is let go that long before the carriage will reach
        // the release height - and until the carriage is actually there, the
        // clamp below holds it at ARM_EARLY_HOLD_ANGLE, a few degrees off
        // RAISE. Those 0.2 s are the difference between the carriage braking
        // for its 35.5 in clamp and sailing through it.
        double releaseHeight = Math.min(SuperstructureConstants.ARM_RELEASE_MIN_HEIGHT,
            targetHeight - SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET);
        Command armReleased = Commands.waitUntil(
            () -> heightAtLeast(releaseHeight - armReleaseLeadSeconds * Math.max(elevator.getVelocity(), 0.0))
                && heightAtMost(SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT));
        DoubleSupplier earlyHold = () -> heightAtLeast(releaseHeight) ? Double.NaN
            : SuperstructureConstants.ARM_EARLY_HOLD_ANGLE;

        // Mid scoring pose (L3): the carriage goes straight to the target at
        // RAISE and the arm sweeps during the last six inches of the climb,
        // rather than starting once the carriage has all but stopped. The
        // last row (25-30 deg) opens at 30.1 in (its 30 in floor plus
        // ARM_CLAMP_HEIGHT_MARGIN), as the carriage arrives.
        if (targetHeight < SuperstructureConstants.L4_ZONE_MIN_HEIGHT) {
            return elevatorTo(targetHeight, climbPace)
                .andThen(armReleased)
                .andThen(armWithCarriage(targetAngle, earlyHold))
                .andThen(settle());
        }
        // High scoring pose (L4): the arm comes down while the carriage
        // climbs behind it, its ceiling rising as the arm passes each row
        // (36 in at 95 deg, 40 at 70, 48 at 40, 51 at 25). Below the
        // final-angle height the arm is held at the stage angle - the one
        // robot-proven gate on the arm side that the table does not imply.
        boolean[] released = {false};
        double[] committed = {SAFE_ANGLE};
        Command armWork = armReleased
            .andThen(Commands.runOnce(() -> released[0] = true))
            .andThen(armWithCarriage(targetAngle,
                () -> !heightAtLeast(releaseHeight) ? SuperstructureConstants.ARM_EARLY_HOLD_ANGLE
                    : heightAtLeast(SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT)
                        ? Double.NaN : SuperstructureConstants.L4_STAGE_ANGLE,
                committed));
        // The last few inches still wait for the arm to reach the scoring
        // angle. The corridor table says 25 deg is clear to 51 in, but the
        // robot says otherwise up there - the claw meets the top bar if the
        // carriage climbs past the pre-top height with the arm still at
        // 25 deg - so the pre-top height stays a hard ceiling until the arm
        // is inside its final gate. Everything below that is continuous.
        //
        // The carriage is not parked at the 33 in station and re-released
        // from there (that would be a stop, then a retarget backwards to the
        // ratchet's first clamp). One ratchet owns it from the first loop:
        // it climbs straight to the clamp the arm's sweep allows (35.5 in
        // while the arm is at RAISE), the arm is released as the carriage
        // passes the release height, and the clamp opens as the arm comes
        // down - the earlier the release, the less the carriage has to slow
        // for that first clamp. Only a carriage that starts above the
        // release window comes down into it first.
        return Commands.deadline(armWork,
                travelWithArm(() -> {
                    if (!released[0]) {
                        // Arm still parked at RAISE: head for the top of the
                        // release window, from below or from above. (A carriage
                        // coming down into the window stops there rather than going
                        // on to the 33 in station and climbing back the 2.5 in;
                        // the arm is released as it arrives.)
                        return SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT - SuperstructureConstants.RATCHET_MARGIN;
                    }
                    return armStrictlyAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)
                        ? targetHeight : SuperstructureConstants.L4_PRE_TOP_HEIGHT;
                },
                // Until the release the arm is parked at RAISE, not sweeping.
                // Clamping the carriage for a sweep that has not started is
                // wrong below 17.5 in, where every row's low band holds the
                // carriage and the sweep's ceiling is the low-box roof: a
                // carriage that is still down there when the arm reaches
                // RAISE (a slow elevator, or L2 -> L4) would stop at 17 in
                // while the arm waits for it to reach the release height.
                // Covered by SuperstructureClampSimTest.
                // ...and from the release on, for the sweep the arm is committed
                // to - the limit it has actually been sent to - plus a
                // lookahead, not the whole sweep to the scoring angle. With
                // the early release the carriage can still be at 17 in, where
                // walking all the way to the scoring angle finds every row's
                // low band and pulls the carriage back to the low-box roof.
                // The lookahead is what keeps the two clamps from waiting on
                // each other: the carriage must already respect the rows the
                // arm is about to be allowed into, or it parks half an inch
                // above the height that would open them.
                () -> released[0]
                    ? Math.max(targetAngle, committed[0] - SuperstructureConstants.ARM_COMMIT_LOOKAHEAD)
                    : SAFE_ANGLE,
                () -> climbPace))
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
            // ...but only if an exit actually moved it. From every other pose
            // (the algae intakes above all - this is the algae hold) the
            // carriage never leaves its setpoint, and re-commanding it to its
            // own measured height would ask the Spark MAX for a zero-length
            // MAXMotion profile - something no other pose ever requests
            // (every other pose is a real move to a preset), and which can
            // leave the elevator rumbling in place. The carriage keeps the
            // setpoint it has.
            return escapeToSafe(stay)
                .andThen(Commands.either(
                    both(stay, SAFE_ANGLE),
                    armTo(SAFE_ANGLE),
                    () -> Math.abs(elevator.getCurrentPosition() - stay) > SuperstructureConstants.RAISE_ARM_PUT_BACK_MIN))
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
     * Autos put a timeout on this (after their own stow) - a timeout
     * around the whole intakeCoral would race the stow's settle timeout and
     * could expire before the rollers ever start.
     */
    public Command intakeRollers() {
        return Commands.sequence(
            Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.CORAL_INTAKE_SPEED), coral),
            Commands.waitUntil(coral::isGamePieceDetected),
            // Explicit stop: the subsystem's auto-stop only fires on the
            // rising edge of detection - if the coral was already latched
            // by the time the rollers started (e.g., it arrived during the
            // stow phase), no edge ever comes and the rollers would run
            // forever without this.
            Commands.runOnce(coral::stopIntake, coral)
        ).handleInterrupt(coral::stopIntake) // Never leave rollers running on interrupt
            .unless(coral::isGamePieceDetected);
    }

    /** Moves to the L1 scoring pose (base height, 100 deg). */
    public Command goToCoralL1() {
        return setGoal(Goal.CORAL_L1)
            .andThen(moveTo(PresetHeights.CORAL_L1, PivotPresetAngles.CORAL_L1));
    }

    /** Moves to the L2 scoring pose (12 in, 12.5 deg). */
    public Command goToCoralL2() {
        return setGoal(Goal.CORAL_L2)
            .andThen(moveTo(PresetHeights.CORAL_L2, PivotPresetAngles.CORAL_L2));
    }

    /** Moves to the L3 scoring pose (30.5 in, 25 deg): climb at RAISE, arm released as the carriage passes 24 in. */
    public Command goToCoralL3() {
        return setGoal(Goal.CORAL_L3)
            .andThen(moveTo(PresetHeights.CORAL_L3, PivotPresetAngles.CORAL_L3));
    }

    /** Moves to the L4 scoring pose (51.5 in, 20 deg): arm released as the carriage passes 24 in, pre-top gated. */
    public Command goToCoralL4() {
        return setGoal(Goal.CORAL_L4)
            .andThen(moveTo(PresetHeights.CORAL_L4, PivotPresetAngles.CORAL_L4));
    }

    /**
     * The roller duty that releases what is held at the given goal. L2 - L4
     * push the coral on through the claw (the intake's direction, faster);
     * L1 sends it back out the way it came in, so its sign is the opposite
     * of both the intake's and the other levels'. Algae leave the way the
     * L2 - L4 coral do.
     */
    static double scoreSpeed(Goal goal) {
        switch (goal) {
            case ALGAE_SCORE:
            case ALGAE_CARRY:
                return CorAlConstants.ALGAE_SCORE_SPEED;
            case CORAL_L1:
                return CorAlConstants.CORAL_L1_SCORE_SPEED;
            default:
                return CorAlConstants.CORAL_SCORE_SPEED;
        }
    }

    /**
     * Ejects the held coral at the current pose: run the rollers at the
     * current goal's {@link #scoreSpeed} for CORAL_EJECT_SECONDS, then stop them.
     * It only handles the rollers and is not gated on the pose; the autos'
     * named commands use it. Teleop scoring goes through {@link #score}.
     */
    public Command ejectCoral() {
        // Deliberately requires NO subsystem: the rollers share the CorAl
        // subsystem with the pivot, so requiring it would make an eject
        // cancel a move in progress (leaving the carriage parked at whatever
        // clamp the ratchet last sent) and let any pose command cancel the
        // eject.
        return Commands.sequence(
            Commands.runOnce(() -> coral.ejectRollers(scoreSpeed(currentGoal))),
            Commands.waitSeconds(SuperstructureConstants.CORAL_EJECT_SECONDS),
            Commands.runOnce(coral::stopIntake)
        ).handleInterrupt(coral::stopIntake); // Never leave rollers running on interrupt
    }

    // ==================================================================
    // Algae
    // ==================================================================

    /** Moves to the low algae intake pose (20.5 in, 160 deg) and runs the rollers inward. */
    public Command intakeAlgaeLow() {
        return setGoal(Goal.ALGAE_LOW)
            .andThen(Commands.runOnce(() -> algaeHeld = true))
            .andThen(moveTo(PresetHeights.ALGAE_LOW_INTAKE, PivotPresetAngles.ALGAE_INTAKE))
            .andThen(Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_INTAKE_SPEED), coral));
    }

    /** Moves to the high algae intake pose (37.5 in, 160 deg) and runs the rollers inward. */
    public Command intakeAlgaeHigh() {
        return setGoal(Goal.ALGAE_HIGH)
            .andThen(Commands.runOnce(() -> algaeHeld = true))
            .andThen(moveTo(PresetHeights.ALGAE_HIGH_INTAKE, PivotPresetAngles.ALGAE_INTAKE))
            .andThen(Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_INTAKE_SPEED), coral));
    }

    /**
     * Holds an algae: light inward roller pressure and the arm at the hold
     * angle (RAISE = 100 deg, the safe travel angle, so the elevator
     * stays free to move anywhere). The elevator is left where it is.
     */
    public Command holdAlgae() {
        return Commands.runOnce(() -> {
                algaeHeld = true;
                coral.setIntakeSpeed(CorAlConstants.ALGAE_HOLD_SPEED);
            }, coral)
            .andThen(raiseArm()); // Hold angle == safe travel angle
    }

    /**
     * Moves to the algae scoring pose (52 in, 105 deg), ejects for
     * ALGAE_EJECT_SECONDS, then stops the rollers (mirroring ejectCoral - without
     * the stop they would spin at 50% duty until another roller command).
     * A one-shot for autos or custom bindings; the operator's barge flow is
     * {@link #goToBarge} followed by the score trigger ({@link #score}).
     */
    public Command scoreAlgae() {
        return setGoal(Goal.ALGAE_SCORE)
            .andThen(moveTo(PresetHeights.ALGAE_SCORE, PivotPresetAngles.ALGAE_SCORE))
            .andThen(Commands.runOnce(() -> coral.ejectRollers(CorAlConstants.ALGAE_SCORE_SPEED), coral))
            .andThen(Commands.waitSeconds(SuperstructureConstants.ALGAE_EJECT_SECONDS))
            .andThen(Commands.runOnce(() -> {
                coral.stopIntake();
                algaeHeld = false; // released: the next home() stows and intakes instead of carrying
            }, coral))
            .handleInterrupt(coral::stopIntake); // Never leave rollers running on interrupt
    }

    // ==================================================================
    // Operator workflow: gated score, auto-home, manual take-over
    // ==================================================================

    // The score gate's tolerances and the back-off distance are
    // SuperstructureConstants.SCORE_READY_* and REEF_BACKOFF_METERS.
    /** FPGA time the pose was first measured reached and still (NaN = not now): the "landed" timer. */
    private double landedSince = Double.NaN;

    /** True while a finished L3/L4 score is waiting for the robot to back away before its exit. */
    private boolean waitingForBackOff = false;

    public boolean isAlgaeHeld() {
        return algaeHeld;
    }

    public boolean isWaitingForBackOff() {
        return waitingForBackOff;
    }

    /** {height, angle} of the pose a goal commands, or null when the goal is not a scoring pose. */
    private static double[] scoringPreset(Goal goal) {
        switch (goal) {
            case CORAL_L1: return new double[] {PresetHeights.CORAL_L1.getHeight(), PivotPresetAngles.CORAL_L1.getAngle()};
            case CORAL_L2: return new double[] {PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle()};
            case CORAL_L3: return new double[] {PresetHeights.CORAL_L3.getHeight(), PivotPresetAngles.CORAL_L3.getAngle()};
            case CORAL_L4: return new double[] {PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle()};
            case ALGAE_SCORE: return new double[] {PresetHeights.ALGAE_SCORE.getHeight(), PivotPresetAngles.ALGAE_SCORE.getAngle()};
            case ALGAE_CARRY: return new double[] {PresetHeights.CORAL_L1.getHeight(), PivotPresetAngles.RAISE.getAngle()};
            default: return null;
        }
    }

    /** True when the last commanded pose is one a game piece is released from. */
    public boolean isScoringGoal() {
        return scoringPreset(currentGoal) != null;
    }

    /**
     * True when the mechanisms are measured at the last commanded scoring
     * pose and have stopped. This is the gate on the score trigger: an early
     * pull waits for it, so it can never interrupt a staged move.
     */
    public boolean readyToScore() {
        double[] preset = scoringPreset(currentGoal);
        if (preset == null) {
            return false;
        }
        double height = Math.max(preset[0], ElevatorConstants.ELEVATOR_ZERO_HEIGHT);
        boolean atHeightAndStill =
            Math.abs(elevator.getCurrentPosition() - height) <= SuperstructureConstants.SCORE_READY_HEIGHT_TOL
            && Math.abs(elevator.getVelocity()) <= SuperstructureConstants.SCORE_READY_MAX_ELEVATOR_SPEED
            && Math.abs(coral.getPivotVelocity()) <= SuperstructureConstants.SCORE_READY_MAX_PIVOT_SPEED;
        double now = Timer.getFPGATimestamp();
        if (!atHeightAndStill) {
            landedSince = Double.NaN;
            return false;
        }
        if (Double.isNaN(landedSince)) {
            landedSince = now;
        }
        double angleError = Math.abs(coral.getPivotAngle() - preset[1]);
        return angleError <= SuperstructureConstants.SCORE_READY_ANGLE_TOL
            || (angleError <= SuperstructureConstants.SCORE_READY_LANDED_ANGLE_TOL
                && now - landedSince >= SuperstructureConstants.SCORE_READY_LANDED_SECONDS);
    }


    /**
     * Releases the game piece at the current scoring pose, then goes home by
     * itself. Bind it behind {@link #readyToScore()}.
     *   - Coral: rollers out for at least CORAL_EJECT_SECONDS and until the
     *     CANrange says the coral has left (for at most
     *     CORAL_EJECT_CLEAR_TIMEOUT_SECONDS more). If it is still detected the
     *     command ends without stowing, so the pose is kept for a second try.
     *     Algae: rollers out for ALGAE_EJECT_SECONDS.
     *   - L3 / L4: the exit waits until the drivetrain has backed away
     *     REEF_BACKOFF_METERS from where it ejected (see the constant).
     *   - Then home(): stow and restart the intake, or carry for an algae.
     * Any other button interrupts it at any point.
     */
    public Command score(Supplier<Pose2d> robotPose) {
        return Commands.defer(() -> {
            Goal scored = currentGoal;
            boolean algae = scored == Goal.ALGAE_SCORE || scored == Goal.ALGAE_CARRY;
            boolean exitSwingsForward = scored == Goal.CORAL_L3 || scored == Goal.CORAL_L4;
            Pose2d ejectedAt = robotPose.get();

            Command rollersOut = Commands.startEnd(
                () -> coral.ejectRollers(scoreSpeed(scored)), // L1 runs the rollers the other way
                coral::stopIntake, coral);
            Command ejectWindow = algae
                ? Commands.waitSeconds(SuperstructureConstants.ALGAE_EJECT_SECONDS)
                : Commands.waitSeconds(SuperstructureConstants.CORAL_EJECT_SECONDS)
                    .andThen(Commands.waitUntil(() -> !coral.isGamePieceDetected())
                        .withTimeout(SuperstructureConstants.CORAL_EJECT_CLEAR_TIMEOUT_SECONDS));
            Command backedOff = exitSwingsForward
                ? Commands.waitUntil(() -> robotPose.get().getTranslation()
                        .getDistance(ejectedAt.getTranslation()) >= SuperstructureConstants.REEF_BACKOFF_METERS)
                    .beforeStarting(() -> waitingForBackOff = true)
                    .finallyDo(() -> waitingForBackOff = false)
                : Commands.none();

            return Commands.deadline(ejectWindow, rollersOut)
                .andThen(Commands.runOnce(() -> algaeHeld = false))
                .andThen(backedOff.andThen(home()).unless(coral::isGamePieceDetected));
        }, Set.of(elevator, coral));
    }

    /**
     * For autos: stow once the drivetrain has moved REEF_BACKOFF_METERS from
     * where this command started. Leaving L3 / L4 swings the claw 9-12 in
     * past the front bumper, so a stow in place against the reef swings it
     * into the reef; run this alongside the departing path instead (an
     * event marker at its start, or a parallel group).
     */
    public Command stowAfterBackingOff(Supplier<Pose2d> robotPose) {
        return Commands.defer(() -> {
            Pose2d from = robotPose.get();
            return Commands.waitUntil(() -> robotPose.get().getTranslation()
                    .getDistance(from.getTranslation()) >= SuperstructureConstants.REEF_BACKOFF_METERS)
                .andThen(stow());
        }, Set.of(elevator, coral));
    }

    /**
     * The one "safe" button: the right home for whatever is held. Empty or
     * holding a coral: stow, and start the intake rollers if empty. Holding
     * an algae: carry it low at the travel angle with holding pressure.
     */
    public Command home() {
        return Commands.either(carryAlgae(), stow().andThen(intakeRollers()), () -> algaeHeld);
    }

    /** Algae carry: holding pressure, base height, travel angle (the L1 pose, which the corridors clear). */
    public Command carryAlgae() {
        return setGoal(Goal.ALGAE_CARRY)
            .andThen(Commands.runOnce(() -> coral.setIntakeSpeed(CorAlConstants.ALGAE_HOLD_SPEED), coral))
            .andThen(moveTo(PresetHeights.CORAL_L1, PivotPresetAngles.RAISE));
    }

    /** Barge pose only (holding pressure kept on); the score trigger releases the algae. */
    public Command goToBarge() {
        return setGoal(Goal.ALGAE_SCORE)
            .andThen(Commands.runOnce(() -> {
                algaeHeld = true;
                coral.setIntakeSpeed(CorAlConstants.ALGAE_HOLD_SPEED);
            }, coral))
            .andThen(moveTo(PresetHeights.ALGAE_SCORE, PivotPresetAngles.ALGAE_SCORE));
    }

    /**
     * Brings both mechanisms to rest where their profiles can actually stop
     * them: the target is the measured position plus the stopping distance
     * v^2 / 2a, so nothing overshoots and comes back the way "hold the
     * current position" does at speed.
     */
    private void freezeNow() {
        elevator.setPosition(stoppingPoint(Pace.FULL), Pace.FULL);
        double w = coral.getPivotVelocity();
        coral.setPivotAngle(coral.getPivotAngle()
            + Math.copySign(w * w / (2.0 * Math.max(coral.maxAcceleration(), 1.0)), w));
    }

    /**
     * Manual take-over, bound whileTrue to a modifier. Pressing it cancels
     * whatever sequence is running and freezes both mechanisms; while it is
     * held the sticks drive them (NO collision interlocks); releasing it
     * holds position. With the modifier up the sticks do nothing at all.
     */
    public Command manualOverride(DoubleSupplier elevatorStick, DoubleSupplier pivotStick) {
        return Commands.startRun(
            this::freezeNow,
            () -> {
                double speed = elevatorStick.getAsDouble();
                if (Math.abs(speed) > ElevatorConstants.ELEVATOR_MANUAL_CONTROL_DEADBAND) {
                    elevator.manualControl(speed);
                } else if (elevator.isInManualMode()) {
                    // Stick released: hold where the carriage can actually stop.
                    // "Hold the measured height" hands the profile a target that
                    // is already behind a moving carriage - it overshoots and
                    // drives back, a bounce at the end of every jog.
                    elevator.setPosition(stoppingPoint(Pace.FULL), Pace.FULL);
                }
                coral.manualPivotControl(pivotStick.getAsDouble());
            }, elevator, coral)
            .finallyDo(() -> {
                if (elevator.isInManualMode()) {
                    elevator.setPosition(stoppingPoint(Pace.FULL), Pace.FULL);
                }
                coral.manualPivotControl(0.0); // captures and holds if the stick was deflected
            });
    }

    /**
     * Rollers at a fixed duty while held, at any pose. Deliberately has no
     * subsystem requirement so it can run inside the manual take-over
     * without cancelling it.
     */
    public Command rollersRaw(double dutyCycle) {
        return Commands.startEnd(() -> coral.setIntakeSpeed(dutyCycle), () -> {
            coral.stopIntake();
            algaeHeld = false;
        });
    }

    // ==================================================================
    // Independent mechanism testing (Elastic Testing tab)
    // ==================================================================
    // These commands move one mechanism and leave the other exactly where
    // it is, so each can be tuned and exercised on its own from the
    // dashboard. They consult the same CAD corridors as the planner - but
    // instead of moving the other mechanism out of the way, an unsafe
    // request is refused with a dashboard notification explaining
    // why. Every command requires only the mechanism it moves, so a test
    // of one never interrupts a test of the other.

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
