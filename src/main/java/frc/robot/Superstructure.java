package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.CorAlConstants;
import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.CorAl;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Elastic;
import frc.robot.util.Tunables;

/**
 * Coordinated elevator + CorAl motion ("superstructure") command factory.
 *
 * The elevator and the CorAl arm can collide with the elevator structure at
 * specific combinations of height and angle (see SuperstructureConstants for
 * the measured contact points). Every position command in the robot goes
 * through this class, whose motion planner picks the FASTEST sequence that
 * stays inside the safe regions, based on where the mechanisms actually are
 * when the command starts:
 *
 *   - "Low box" moves (both start and end below LOW_TRAVEL_MAX_HEIGHT with
 *     the arm at/above ARM_CLEAR_MIN_ANGLE) run direct - the arm and
 *     elevator move together with no safe-angle excursion. Base to L2, base
 *     to L1, and base to the low algae intake are all single direct motions.
 *   - Moves that leave the low box travel with the arm at the safe angle
 *     (90 degrees). When ascending, the elevator gets a bounded head start
 *     (up to the highest height that is safe for the CURRENT arm angle)
 *     while the arm swings up, so the swing costs little or no time.
 *   - Handoff overlaps: above the low box the arm cannot rotate below 90
 *     degrees in place without sweeping into the second-stage tube, so the
 *     final rotation overlaps the last part of the climb - past
 *     MID_HANDOFF_HEIGHT for L3 (29", 22.5 deg) and HIGH_HANDOFF_HEIGHT for
 *     L4 (52.5", 45 deg). Leaving those poses mirrors the overlap with the
 *     arm swinging up during the initial descent.
 *   - Stowing overlaps the final tuck: the arm starts rotating to 0 as soon
 *     as the descending elevator passes ARM_TUCK_MAX_HEIGHT.
 *
 * All overlaps are gated on measured state (heights and angles), never on
 * timing, so they remain safe if a mechanism runs slower than expected.
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
    // State predicates
    // ==================================================================

    /** True when the arm is at/above the safe travel angle. */
    private boolean armAtOrAboveSafe() {
        return coral.getPivotAngle()
            >= SAFE_ANGLE - SuperstructureConstants.SAFE_ANGLE_TOLERANCE;
    }

    /** True when the arm has cleared the static elevator part (>= ~5 deg). */
    private boolean armClearOfStaticPart() {
        return coral.getPivotAngle()
            >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE - CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }

    /** True when both mechanisms have settled at their commanded targets. */
    private boolean atTargets() {
        return elevator.isAtTargetPosition() && coral.isAtTargetAngle();
    }

    /**
     * Final settle wait for a planned move, with a timeout so a mechanism
     * that stalls just outside its at-target tolerance cannot deadlock the
     * command (and with it the operator's default manual controls). Timing
     * out is safe: the latched closed-loop setpoints keep holding position.
     * Intermediate SAFETY gates deliberately do not get this treatment.
     */
    private Command settle() {
        return Commands.waitUntil(this::atTargets)
            .withTimeout(SuperstructureConstants.SETTLE_TIMEOUT_SECONDS);
    }

    // ==================================================================
    // Motion planner
    // ==================================================================

    /**
     * Returns a command that moves to the given pose safely and as fast as
     * the safe regions allow. Deferred so the plan is built from the REAL
     * mechanism state at the moment the command starts (not at robot init,
     * and not at whatever pose the previous command assumed).
     */
    private Command moveTo(PresetHeights height, PivotPresetAngles angle) {
        return Commands.defer(() -> planMove(height.getHeight(), angle.getAngle()),
            Set.of(elevator, coral));
    }

    /** Builds the motion sequence for a target pose from the current state. */
    private Command planMove(double targetHeight, double targetAngle) {
        double currentHeight = elevator.getCurrentPosition();
        double currentAngle = coral.getPivotAngle();

        // ---- Case 1: the whole move stays inside the low box ----
        // Below LOW_TRAVEL_MAX_HEIGHT every arm angle >= ARM_CLEAR_MIN_ANGLE
        // is contact-free, so the two mechanisms move together. The only
        // gate: if the arm starts fully tucked, let it clear the static part
        // (a ~5 degree swing) before the elevator lifts off the base.
        if (currentHeight <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT
                && targetHeight <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT
                && targetAngle >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE) {
            return Commands.sequence(
                Commands.runOnce(() -> coral.setPivotAngle(targetAngle), coral),
                Commands.waitUntil(this::armClearOfStaticPart),
                Commands.runOnce(() -> elevator.setPosition(targetHeight), elevator),
                settle()
            );
        }

        // ---- Case 2: tuck target (BASE) from inside the low box ----
        // A tucked arm is only allowed near the base, but the descent to get
        // there is safe at ANY arm angle while inside the low box - so skip
        // the 90-degree excursion entirely: send the elevator down, then
        // tuck once the carriage is below the tuck limit. From the base pose
        // itself this reduces to (almost) a no-op instead of a pointless
        // 0 -> 90 -> 0 arm round trip. The currentAngle guard excludes the
        // (manual-control-only) pathological state of a tucked arm above the
        // tuck limit, which routes to the general path's escape instead.
        if (targetAngle < SuperstructureConstants.ARM_CLEAR_MIN_ANGLE
                && currentHeight <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT
                && targetHeight <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT
                && (currentAngle >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE
                        - CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR
                    || currentHeight <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT)) {
            return Commands.sequence(
                Commands.runOnce(() -> elevator.setPosition(targetHeight), elevator),
                Commands.waitUntil(() ->
                    elevator.getCurrentPosition() <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT),
                Commands.runOnce(() -> coral.setPivotAngle(targetAngle), coral),
                settle()
            );
        }

        // ---- General path: escape the current pose, travel at the safe
        //      angle, finish at the target ----
        return escapeCurrentPose(targetHeight, targetAngle)
            .andThen(travelAndFinish(targetHeight, targetAngle));
    }

    /**
     * Gets the arm to/above the safe travel angle from wherever it is now,
     * as cheaply as possible:
     *  - already safe: nothing to do;
     *  - high pose with the arm below safe (e.g. sitting at L4): rotating up
     *    in place would strike the second-stage tube, so descend toward the
     *    handoff height while the arm swings up (mirror of the L4 approach);
     *  - ascending move: give the elevator a head start toward the target,
     *    bounded by the highest height that is safe for the current arm
     *    angle, so the swing overlaps useful travel;
     *  - otherwise: swing up in place.
     *
     * The arm is sent to the FINAL angle when that angle is itself at/above
     * safe (everything >= 90 degrees is contact-free at every height), so a
     * free-zone target like the algae intake sweeps up in one continuous
     * motion instead of decelerating at 90 and re-accelerating when
     * travelAndFinish retargets it.
     */
    private Command escapeCurrentPose(double targetHeight, double targetAngle) {
        double currentHeight = elevator.getCurrentPosition();
        double currentAngle = coral.getPivotAngle();

        // Where the arm swings during the escape: the final angle if it is
        // already in the free zone, otherwise the safe travel angle.
        double escapeAngle = Math.max(SAFE_ANGLE, targetAngle);

        // Already at/above the safe angle: no escape needed
        if (currentAngle >= SAFE_ANGLE - SuperstructureConstants.SAFE_ANGLE_TOLERANCE) {
            return Commands.none();
        }

        // Arm below safe above the low box (the L3/L4 scoring poses):
        // in-place rotation would sweep into the second-stage tube, so
        // mirror the approach overlap - descend toward the pose's handoff
        // height while the arm swings up. Never commands upward.
        if (currentHeight > SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT) {
            // Handoff heights are live-tunable (Tunables -> Preferences) and
            // read here at plan time, so a dashboard edit applies to the very
            // next button press.
            double highHandoff = Tunables.highHandoffHeight();
            double handoff = currentHeight > highHandoff ? highHandoff : Tunables.midHandoffHeight();
            double escapeFloor = Math.min(handoff, currentHeight);
            return Commands.runOnce(() -> {
                    elevator.setPosition(escapeFloor);
                    coral.setPivotAngle(escapeAngle);
                }, elevator, coral)
                .andThen(Commands.waitUntil(this::armAtOrAboveSafe));
        }

        // Ascending: head start up to the height that is safe at the arm's
        // MEASURED angle, re-evaluated every loop so the ceiling ratchets up
        // as the arm swings (tuck limit while fully tucked, low box roof once
        // the arm clears the static part). Computing the bound once at plan
        // time would park the elevator at the stale bound waiting for the
        // arm - a stop-and-go stutter; here the target only ever moves up,
        // and MAXMotion re-profiles each retarget from the current motion
        // state, so the carriage keeps climbing smoothly until the arm is
        // safe and travelAndFinish hands it the final height.
        // Never commands downward.
        if (targetHeight > currentHeight) {
            return Commands.runOnce(() -> coral.setPivotAngle(escapeAngle), coral)
                .andThen(Commands.run(() -> {
                    double ceiling = armClearOfStaticPart()
                        ? SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT
                        : SuperstructureConstants.ARM_TUCK_MAX_HEIGHT;
                    elevator.setPosition(Math.max(Math.min(targetHeight, ceiling),
                        elevator.getCurrentPosition()));
                }, elevator).until(this::armAtOrAboveSafe));
        }

        // Descending (or staying): swing up in place. Descending with the
        // arm low is NOT safe in general (the 5-degree tube contact zone is
        // bidirectional), so no downward head start.
        return Commands.runOnce(() -> coral.setPivotAngle(escapeAngle), coral)
            .andThen(Commands.waitUntil(this::armAtOrAboveSafe));
    }

    /**
     * Travels to the target height and settles at the target angle, given
     * the arm is already at/above the safe angle. Picks the finishing
     * strategy by target type.
     */
    private Command travelAndFinish(double targetHeight, double targetAngle) {
        // Target angle in the free zone (>= safe angle): the elevator and
        // arm move simultaneously the whole way
        if (targetAngle >= SAFE_ANGLE - SuperstructureConstants.SAFE_ANGLE_TOLERANCE) {
            return Commands.sequence(
                Commands.runOnce(() -> {
                    elevator.setPosition(targetHeight);
                    coral.setPivotAngle(targetAngle);
                }, elevator, coral),
                settle()
            );
        }

        // Full tuck (0 degrees): descend at the safe angle and start the
        // tuck as soon as the carriage passes the tuck limit
        if (targetAngle < SuperstructureConstants.ARM_CLEAR_MIN_ANGLE) {
            return Commands.sequence(
                Commands.runOnce(() -> elevator.setPosition(targetHeight), elevator),
                Commands.waitUntil(() ->
                    elevator.getCurrentPosition() <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT),
                Commands.runOnce(() -> coral.setPivotAngle(targetAngle), coral),
                settle()
            );
        }

        // Scoring pose above the low box with the arm below safe (L3, L4):
        // overlap the final rotation with the last part of the climb, past
        // the pose's handoff height, so the mechanism sweeps in behind the
        // second-stage tube instead of into it
        if (targetHeight > SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT) {
            double highHandoff = Tunables.highHandoffHeight();
            double handoff = targetHeight > highHandoff ? highHandoff : Tunables.midHandoffHeight();
            return Commands.sequence(
                Commands.runOnce(() -> elevator.setPosition(targetHeight), elevator),
                Commands.waitUntil(() -> elevator.getCurrentPosition() >= handoff),
                Commands.runOnce(() -> coral.setPivotAngle(targetAngle), coral),
                settle()
            );
        }

        // Scoring pose inside the low box (L2 at 12", 5 deg), reached from
        // above: descend at the safe angle and start the final rotation as
        // soon as the carriage is back inside the low box
        return Commands.sequence(
            Commands.runOnce(() -> elevator.setPosition(targetHeight), elevator),
            Commands.waitUntil(() ->
                elevator.getCurrentPosition() <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT),
            Commands.runOnce(() -> coral.setPivotAngle(targetAngle), coral),
            settle()
        );
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
     * Swings the arm to the safe travel angle at the current height,
     * automatically handling a high pose that needs the overlapped escape.
     */
    public Command raiseArm() {
        return Commands.defer(() ->
            escapeCurrentPose(elevator.getCurrentPosition(), SAFE_ANGLE)
                .andThen(Commands.runOnce(() -> coral.setPivotAngle(SAFE_ANGLE), coral))
                .andThen(Commands.waitUntil(coral::isAtTargetAngle)),
            Set.of(elevator, coral));
    }

    /**
     * Full coral intake: stow to the base pose, then run the rollers until
     * the CANrange confirms a game piece (the subsystem stops them
     * automatically on confirmation). Skipped entirely if a piece is
     * already held.
     */
    public Command intakeCoral() {
        return Commands.sequence(
            stow(),
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

    /** Moves to the L1 scoring pose (0 in, 100 degrees). */
    public Command goToCoralL1() {
        return setGoal(Goal.CORAL_L1)
            .andThen(moveTo(PresetHeights.CORAL_L1, PivotPresetAngles.CORAL_L1));
    }

    /** Moves to the L2 scoring pose (12 in, 5 degrees). */
    public Command goToCoralL2() {
        return setGoal(Goal.CORAL_L2)
            .andThen(moveTo(PresetHeights.CORAL_L2, PivotPresetAngles.CORAL_L2));
    }

    /** Moves to the L3 scoring pose (29 in, 22.5 degrees) with the mid handoff overlap. */
    public Command goToCoralL3() {
        return setGoal(Goal.CORAL_L3)
            .andThen(moveTo(PresetHeights.CORAL_L3, PivotPresetAngles.CORAL_L3));
    }

    /** Moves to the L4 scoring pose (52.5 in, 45 degrees) with the high handoff overlap. */
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
     * angle (90 degrees - also the safe travel angle, so the elevator stays
     * free to move). The elevator is left where it is.
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
    // dashboard. They still consult the same safe-region model as the
    // planner - but instead of moving the other mechanism out of the way,
    // an unsafe request is simply REFUSED with a dashboard notification
    // explaining why. Every command requires only the mechanism it moves,
    // so the operator's manual control of the other one keeps working.

    /**
     * True if the elevator can move from its current height to the target
     * with the arm held exactly where it is now.
     */
    public boolean isElevatorOnlyMoveSafe(double targetHeight) {
        double height = elevator.getCurrentPosition();
        if (armAtOrAboveSafe()) {
            return true; // Free zone: clear at every height
        }
        if (armClearOfStaticPart()) {
            // Low box: any height up to its roof is clear at this angle
            return height <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT
                && targetHeight <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT;
        }
        // Fully tucked: only the region below the tuck limit is clear
        return height <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT
            && targetHeight <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT;
    }

    /**
     * True if the arm can rotate from its current angle to the target with
     * the elevator held exactly where it is now.
     */
    public boolean isPivotOnlyMoveSafe(double targetAngle) {
        double height = elevator.getCurrentPosition();
        if (height <= SuperstructureConstants.ARM_TUCK_MAX_HEIGHT) {
            return true; // Near the base every angle is clear
        }
        if (height <= SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT) {
            // Inside the low box: anything except a full tuck
            return targetAngle >= SuperstructureConstants.ARM_CLEAR_MIN_ANGLE;
        }
        // Above the low box: rotating below the safe angle in place sweeps
        // into the second-stage tube, so both ends must be in the free zone
        return armAtOrAboveSafe()
            && targetAngle >= SAFE_ANGLE - SuperstructureConstants.SAFE_ANGLE_TOLERANCE;
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
                    String.format("%.1f in is not reachable with the arm at %.0f deg - raise the arm "
                        + "(Pivot Setpoint 90 + Pivot Go, or the Raise button) first.",
                        target, coral.getPivotAngle())));
                return Commands.none();
            }
            return Commands.runOnce(() -> elevator.setPosition(target), elevator);
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
                    String.format("%.0f deg is not reachable with the elevator at %.1f in - lower the "
                        + "elevator into the low box (below %.0f in) first, or use a preset button.",
                        target, elevator.getCurrentPosition(),
                        SuperstructureConstants.LOW_TRAVEL_MAX_HEIGHT)));
                return Commands.none();
            }
            return Commands.runOnce(() -> coral.setPivotAngle(target), coral);
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
