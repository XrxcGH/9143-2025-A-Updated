package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
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
                + "L3 up: rotate %.0f in below target; down: lift to %.0f, descend at RAISE",
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
            return SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT;
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
    public static double ceilingForSweep(double from, double to, double height) {
        double ceiling = ElevatorConstants.ELEVATOR_MAX_POSITION;
        // Walk FROM the arm's current angle toward its destination. The first
        // sample is the current pose, which is clear; the walk stops at the
        // first angle whose band does not contain this height, because the
        // arm cannot pass that angle until the carriage has moved - and by
        // then this is recomputed.
        double step = to >= from ? SWEEP_STEP : -SWEEP_STEP;
        double angle = from;
        while (true) {
            double rowCeiling = bandCeiling(angle, height);
            if (Double.isNaN(rowCeiling)) {
                break;
            }
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
        double step = to >= from ? SWEEP_STEP : -SWEEP_STEP;
        double angle = from;
        while (true) {
            double rowFloor = bandFloor(angle, height);
            if (Double.isNaN(rowFloor)) {
                break;
            }
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

    /**
     * Climbs toward {@code target} as fast as the arm allows: every loop the
     * carriage is given the target, clamped below the ceiling for the sweep
     * the arm has left to do, and never below where it already is.
     */
    private Command climbWithArm(DoubleSupplier targetSupplier, DoubleSupplier armDestination) {
        return Commands.run(() -> {
            double target = targetSupplier.getAsDouble();
            double height = elevator.getCurrentPosition();
            double raw = ceilingForSweep(coral.getPivotAngle(), armDestination.getAsDouble(), height);
            // The margin holds the carriage short of a ceiling it is being
            // clamped AT, but never short of the target itself: the scoring
            // poses sit at the top of their band by design (L4 is 52.5 in a
            // band that ends at 52.5), so subtracting a margin from those
            // would leave the carriage permanently half an inch low.
            double ceiling = Math.max(raw - SuperstructureConstants.RATCHET_MARGIN, Math.min(target, raw));
            elevator.setPosition(Math.max(Math.min(target, ceiling), height));
        }, elevator);
    }

    /** Mirror of {@link #climbWithArm}: descends as fast as the arm allows. */
    private Command descendWithArm(DoubleSupplier targetSupplier, DoubleSupplier armDestination) {
        return Commands.run(() -> {
            double target = targetSupplier.getAsDouble();
            double height = elevator.getCurrentPosition();
            double raw = floorForSweep(coral.getPivotAngle(), armDestination.getAsDouble(), height);
            double floor = Math.min(raw + SuperstructureConstants.RATCHET_MARGIN, Math.max(target, raw));
            elevator.setPosition(Math.min(Math.max(target, floor), height));
        }, elevator);
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
        boolean startsLow = h0 <= SuperstructureConstants.LOW_BOX_ROOF;
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
        return lowerFirst.andThen(escapeToSafe(target)).andThen(approach(target, targetAngle));
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
            return Commands.none(); // already free to travel
        }
        if (h0 >= SuperstructureConstants.L4_ZONE_MIN_HEIGHT && a0 < SuperstructureConstants.BAND_PASS_MIN_ANGLE) {
            return leaveHighPose(targetHeight < SuperstructureConstants.L4_STATION_HEIGHT);
        }
        if (h0 > SuperstructureConstants.LOW_BOX_ROOF && a0 < SuperstructureConstants.BAND_PASS_MIN_ANGLE) {
            return leaveMidPose(h0, targetHeight < SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT);
        }
        if (h0 > SuperstructureConstants.LOW_BOX_ROOF) {
            // Arm between the band-pass and safe-travel angles above the low
            // box (e.g. 90 deg at 38 in after manual control): rotating on
            // toward RAISE moves the claw's rear away from the tube.
            return armTo(SAFE_ANGLE)
                .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE)));
        }

        // In the low box. Swing up; when the move is upward, let the carriage
        // climb as far as the arm's MEASURED angle allows, re-evaluated every
        // loop so the ceiling ratchets up as the arm swings (tuck limit ->
        // low box roof -> mid corridor -> anything). MAXMotion re-profiles
        // each retarget from the current motion state, so the carriage keeps
        // climbing smoothly. Never commands downward.
        Command swing = armTo(SAFE_ANGLE);
        if (targetHeight > h0) {
            return swing.andThen(Commands.run(() -> {
                double ceiling = climbCeiling(coral.getPivotAngle());
                elevator.setPosition(Math.max(Math.min(targetHeight, ceiling), elevator.getCurrentPosition()));
            }, elevator).until(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE)));
        }
        return swing.andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE)));
    }

    /**
     * Leaves a mid-height scoring pose (L3): lift to the return-lift height
     * while the arm swings to RAISE, and only hand the carriage on once the
     * arm is free to travel. Descending any earlier sweeps the claw into
     * the middle-stage top tube.
     */
    private Command leaveMidPose(double currentHeight, boolean descendingNext) {
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
        return elevatorTo(lift)
            .andThen(Commands.deadline(armWork,
                descendWithArm(() -> armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE)
                        ? ElevatorConstants.ELEVATOR_ZERO_HEIGHT : lift,
                    () -> SAFE_ANGLE)));
    }

    /**
     * Leaves the high scoring pose (L4): drop first at the L4 angle, stage
     * the arm to 45 deg while low enough, drop to the station, swing to
     * RAISE below the safe-rotate height, then the carriage is free.
     */
    private Command leaveHighPose(boolean descendingNext) {
        double release = descendingNext ? SuperstructureConstants.BAND_PASS_MIN_ANGLE
            : SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE;
        // The carriage drops first (the top is only clear at the L4 angle),
        // and once it is low enough for the arm to start swinging back, the
        // two run together: the arm sweeps up to RAISE while the carriage
        // keeps descending, its floor falling as the arm rises (39 in at 45
        // deg, 0 past 75). One continuous drop instead of drop-rotate-drop.
        Command armWork = armTo(SuperstructureConstants.L4_RETURN_STAGE_ANGLE)
            .andThen(Commands.waitUntil(() -> armAtLeast(SuperstructureConstants.L4_RETURN_STAGE_DONE_ANGLE)))
            .andThen(armTo(SAFE_ANGLE))
            .andThen(Commands.waitUntil(() -> armAtLeast(release)));
        // Mirror of the climb: continuous down to the station, and below it
        // only once the arm has cleared band A, which is what the staged
        // version waited for.
        return elevatorTo(SuperstructureConstants.L4_RETURN_DROP_HEIGHT)
            .andThen(Commands.waitUntil(() -> heightAtMost(SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT)))
            .andThen(Commands.deadline(armWork,
                descendWithArm(() -> {
                    if (armAtLeast(SuperstructureConstants.BAND_PASS_MIN_ANGLE)) {
                        return ElevatorConstants.ELEVATOR_ZERO_HEIGHT;   // band A cleared: all the way down
                    }
                    if (armAtLeast(SuperstructureConstants.L4_RETURN_STAGE_DONE_ANGLE)) {
                        return SuperstructureConstants.L4_STATION_HEIGHT; // staged: down to the station
                    }
                    return SuperstructureConstants.L4_RETURN_DROP_HEIGHT; // arm still near the L4 angle
                }, () -> SAFE_ANGLE)));
    }

    /**
     * Travels to the target and settles at the target angle, given the arm
     * is at/above the safe travel angle. Picks the strategy by target type.
     */
    private Command approach(double targetHeight, double targetAngle) {
        // Tuck target (BASE) from above: descend at RAISE, tuck below the limit
        if (targetAngle < SuperstructureConstants.ARM_CLEAR_MIN_ANGLE) {
            return elevatorTo(targetHeight)
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
        // Mid scoring pose (L3): travel at RAISE, final rotation only within
        // MID_POSE_ROTATE_BELOW_TARGET of the target (coming from above as
        // well as below - the pose's band is narrow)
        if (targetHeight < SuperstructureConstants.L4_ZONE_MIN_HEIGHT) {
            double lo = targetHeight - SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET;
            double hi = targetHeight + SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET;
            return elevatorTo(targetHeight)
                .andThen(Commands.waitUntil(() -> heightAtLeast(lo) && heightAtMost(hi)))
                .andThen(armTo(targetAngle))
                .andThen(settle());
        }
        // High scoring pose (L4): the carriage rises to the rotation station
        // (the only window where 25-100 deg is clear), the arm starts its
        // rotation as the carriage ARRIVES there - the gate is the window,
        // not a settled height - and from then on the two run together: the
        // arm continues down to the scoring angle while the carriage climbs
        // continuously behind it, its ceiling rising as the arm comes down
        // (36 in at 95 deg, 40 at 70, 48 at 40, 52.5 at 25). No second stop.
        double stageAngle = Math.min(targetAngle, SuperstructureConstants.L4_STAGE_ANGLE);
        Command armWork = armTo(stageAngle)
            .andThen(Commands.waitUntil(() -> heightAtLeast(SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT)))
            .andThen(armTo(targetAngle))
            .andThen(Commands.waitUntil(coral::isAtTargetAngle)
                .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS));
        // The last few inches still wait for the arm to REACH the scoring
        // angle. The corridor table says 25 deg is clear to 51 in, but the
        // robot says otherwise up there - L4 caught the top bar when the
        // carriage was allowed to climb past the pre-top height with the arm
        // still at 25 - so the pre-top height stays a hard ceiling until the
        // arm is inside its final gate, exactly as the staged version had it.
        // Everything below that is continuous.
        return elevatorTo(SuperstructureConstants.L4_STATION_HEIGHT)
            .andThen(Commands.waitUntil(() -> heightAtLeast(SuperstructureConstants.L4_ROTATE_START_HEIGHT)
                && heightAtMost(SuperstructureConstants.L4_STATION_HEIGHT + 2.0)))
            .andThen(Commands.deadline(armWork,
                climbWithArm(() -> armAtMost(SuperstructureConstants.L4_FINAL_GATE_ANGLE)
                        ? targetHeight : SuperstructureConstants.L4_PRE_TOP_HEIGHT,
                    () -> targetAngle)))
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
        return Commands.defer(() ->
            escapeToSafe(elevator.getCurrentPosition())
                .andThen(armTo(SAFE_ANGLE))
                .andThen(Commands.waitUntil(coral::isAtTargetAngle)
                    .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS)),
            Set.of(elevator, coral));
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
