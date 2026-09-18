package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.CorAlConstants.PivotPresetAngles;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.PresetHeights;
import frc.robot.Constants.SuperstructureConstants;

/**
 * Pins the CAD-derived free corridors and the staged sequences to each
 * other: if someone edits a station height, a preset, or the table, this
 * fails before the robot does. Pure logic - no HAL.
 */
class SuperstructureCorridorTest {

    private static final double BASE = ElevatorConstants.ELEVATOR_ZERO_HEIGHT;

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
        assertTrue(Superstructure.poseClear(BASE, PivotPresetAngles.BASE.getAngle()));
        assertTrue(Superstructure.poseClear(BASE, PivotPresetAngles.CORAL_L1.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle()));
        // L2 keeps its margin even when the chain lets the arm sag 2.5 deg
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle() - 2.5));
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L3.getHeight(), PivotPresetAngles.CORAL_L3.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.ALGAE_LOW_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.ALGAE_HIGH_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.ALGAE_SCORE.getHeight(), PivotPresetAngles.ALGAE_SCORE.getAngle()));
        // RAISE is the travel angle: clear from the base to the top
        assertTrue(Superstructure.elevatorPathClear(BASE, ElevatorConstants.ELEVATOR_MAX_POSITION,
            PivotPresetAngles.RAISE.getAngle()));
        // Nothing is inside a band any more, so the startup audit is silent.
        assertEquals("", Superstructure.presetAuditMessage());
    }

    @Test
    void ratchetedCarriageTargetsStayInTheCorridorsAndArrive() {
        double margin = SuperstructureConstants.RATCHET_MARGIN;

        // L4 climb: the arm sweeps from RAISE down to the scoring angle
        // while the carriage follows its ceiling up from the station.
        double height = SuperstructureConstants.L4_STATION_HEIGHT;
        double goal = PresetHeights.CORAL_L4.getHeight();
        double finalAngle = PivotPresetAngles.CORAL_L4.getAngle();
        for (double angle = PivotPresetAngles.RAISE.getAngle(); angle >= finalAngle; angle -= 0.5) {
            double raw = Superstructure.ceilingForSweep(angle, finalAngle, height);
            double ceiling = Math.max(raw - margin, Math.min(goal, raw));
            double target = Math.max(Math.min(goal, ceiling), height);
            assertTrue(Superstructure.poseClear(target, angle),
                String.format("L4 climb would command %.2f in at %.1f deg", target, angle));
            assertTrue(Superstructure.elevatorPathClear(height, target, angle),
                String.format("L4 climb path to %.2f in is blocked at %.1f deg", target, angle));
            height = target;
        }
        assertEquals(goal, height, 0.75, "the carriage should reach the L4 height as the arm arrives");

        // L4 return: the arm sweeps up to RAISE while the carriage follows
        // its floor down from the drop height.
        height = SuperstructureConstants.L4_RETURN_DROP_HEIGHT;
        for (double angle = finalAngle; angle <= PivotPresetAngles.RAISE.getAngle(); angle += 0.5) {
            double raw = Superstructure.floorForSweep(angle, PivotPresetAngles.RAISE.getAngle(), height);
            double floor = Math.min(raw + margin, Math.max(BASE, raw));
            double target = Math.min(Math.max(BASE, floor), height);
            assertTrue(Superstructure.poseClear(target, angle),
                String.format("L4 return would command %.2f in at %.1f deg", target, angle));
            assertTrue(Superstructure.elevatorPathClear(height, target, angle),
                String.format("L4 return path to %.2f in is blocked at %.1f deg", target, angle));
            height = target;
        }
        assertTrue(height <= SuperstructureConstants.LOW_BOX_ROOF,
            "the carriage should be down in the low box by the time the arm reaches RAISE, was " + height);

        // L3 return: same, from the lift height.
        height = SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT;
        for (double angle = PivotPresetAngles.CORAL_L3.getAngle();
                angle <= PivotPresetAngles.RAISE.getAngle(); angle += 0.5) {
            double raw = Superstructure.floorForSweep(angle, PivotPresetAngles.RAISE.getAngle(), height);
            double floor = Math.min(raw + margin, Math.max(BASE, raw));
            double target = Math.min(Math.max(BASE, floor), height);
            assertTrue(Superstructure.poseClear(target, angle),
                String.format("L3 return would command %.2f in at %.1f deg", target, angle));
            height = target;
        }
        assertTrue(height <= SuperstructureConstants.LOW_BOX_ROOF,
            "the carriage should be down in the low box by the time the arm reaches RAISE, was " + height);
    }

    @Test
    void stagedSequenceWaypointsAreInside() {
        // Low box: anything from the clear angle to the band-pass angle, up to the roof
        for (double a = SuperstructureConstants.ARM_CLEAR_MIN_ANGLE + 2.0; a <= SuperstructureConstants.BAND_PASS_MIN_ANGLE; a += 2.5) {
            assertTrue(Superstructure.elevatorPathClear(BASE, SuperstructureConstants.LOW_BOX_ROOF, a), "low box at " + a);
        }
        // Tuck zone
        assertTrue(Superstructure.elevatorPathClear(BASE, SuperstructureConstants.ARM_TUCK_MAX_HEIGHT, 0.0));
        // Band-pass angle and up may climb to the mid corridor ceiling
        for (double a = SuperstructureConstants.BAND_PASS_MIN_ANGLE; a < SuperstructureConstants.SAFE_TRAVEL_MIN_ANGLE; a += 2.5) {
            assertTrue(Superstructure.elevatorPathClear(BASE + 3.0, SuperstructureConstants.MID_CORRIDOR_MAX_HEIGHT, a), "mid corridor at " + a);
        }
        // L4 station: every angle from the stage angle to RAISE is clear across the rotate window
        for (double h = SuperstructureConstants.L4_ROTATE_START_HEIGHT; h <= SuperstructureConstants.L4_STATION_HEIGHT + 1.0; h += 0.5) {
            assertTrue(Superstructure.pivotPathClear(SuperstructureConstants.L4_STAGE_ANGLE, PivotPresetAngles.RAISE.getAngle(), h),
                "station rotation at " + h);
        }
        // L4 climb at the stage angle (and up to the stage-done angle) to the pre-top height
        for (double a = SuperstructureConstants.L4_STAGE_ANGLE; a <= SuperstructureConstants.L4_STAGE_DONE_ANGLE; a += 1.0) {
            assertTrue(Superstructure.elevatorPathClear(SuperstructureConstants.L4_STATION_HEIGHT, SuperstructureConstants.L4_PRE_TOP_HEIGHT, a),
                "L4 climb at " + a);
        }
        // L4 finish: from the final-angle height to the top at the L4 angle and the gate angle
        assertTrue(Superstructure.elevatorPathClear(SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT,
            PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle()));
        assertTrue(Superstructure.elevatorPathClear(SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT,
            PresetHeights.CORAL_L4.getHeight(), SuperstructureConstants.L4_FINAL_GATE_ANGLE));
        assertTrue(Superstructure.pivotPathClear(SuperstructureConstants.L4_STAGE_ANGLE, PivotPresetAngles.CORAL_L4.getAngle(),
            SuperstructureConstants.L4_FINAL_ANGLE_MIN_HEIGHT));
        // L4 return: drop at the L4 angle, stage to 45 below the rotate-max height, station, RAISE below the safe-rotate height
        assertTrue(Superstructure.elevatorPathClear(PresetHeights.CORAL_L4.getHeight(), SuperstructureConstants.L4_RETURN_DROP_HEIGHT,
            PivotPresetAngles.CORAL_L4.getAngle()));
        for (double h = SuperstructureConstants.L4_RETURN_DROP_HEIGHT; h <= SuperstructureConstants.L4_RETURN_ROTATE_MAX_HEIGHT; h += 0.5) {
            assertTrue(Superstructure.pivotPathClear(PivotPresetAngles.CORAL_L4.getAngle(),
                SuperstructureConstants.L4_RETURN_STAGE_ANGLE + SuperstructureConstants.SAFE_ANGLE_TOLERANCE, h), "L4 return stage at " + h);
        }
        assertTrue(Superstructure.elevatorPathClear(SuperstructureConstants.L4_RETURN_DROP_HEIGHT, SuperstructureConstants.L4_STATION_HEIGHT,
            SuperstructureConstants.L4_RETURN_STAGE_ANGLE));
        for (double h = SuperstructureConstants.L4_STATION_HEIGHT; h <= SuperstructureConstants.L4_RETURN_SAFE_ROTATE_MAX_HEIGHT; h += 0.5) {
            assertTrue(Superstructure.pivotPathClear(SuperstructureConstants.L4_RETURN_STAGE_ANGLE,
                SuperstructureConstants.BAND_PASS_MIN_ANGLE, h), "L4 return swing (to band-pass) at " + h);
        }
        assertTrue(Superstructure.pivotPathClear(SuperstructureConstants.L4_RETURN_STAGE_ANGLE, PivotPresetAngles.RAISE.getAngle(),
            SuperstructureConstants.L4_STATION_HEIGHT));
        // L3 approach: the RAISE -> L3 rotation is clear across the whole rotate window
        for (double h = PresetHeights.CORAL_L3.getHeight() - SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET;
                h <= PresetHeights.CORAL_L3.getHeight() + SuperstructureConstants.MID_POSE_ROTATE_BELOW_TARGET; h += 0.25) {
            assertTrue(Superstructure.pivotPathClear(PivotPresetAngles.CORAL_L3.getAngle(), PivotPresetAngles.RAISE.getAngle(), h),
                "L3 rotation at " + h);
        }
        // L3 return lift window: the L3 angle up to RAISE clear at the lift height
        assertTrue(Superstructure.pivotPathClear(PivotPresetAngles.CORAL_L3.getAngle(), PivotPresetAngles.RAISE.getAngle(),
            SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT));
        // Algae: the stage angle is clear from the base; the intake angle needs the minimum height
        assertTrue(Superstructure.elevatorPathClear(BASE, ElevatorConstants.ELEVATOR_MAX_POSITION, SuperstructureConstants.HIGH_ANGLE_STAGE));
        assertTrue(Superstructure.elevatorPathClear(SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_POSITION,
            PivotPresetAngles.ALGAE_INTAKE.getAngle()));
    }

    /**
     * The L3 <-> L4 shortcut: both directions have an overlap to turn in,
     * every leg of both is inside the corridors, and no other pair of
     * operator poses is short-cut - the low box and the upper corridor do
     * not overlap at any scoring angle, and the rest are at travel angles
     * already. Also pins the L4 pose off the top of travel.
     */
    @Test
    void directTransferShortCutsL3AndL4AndNothingElse() {
        double l3 = PresetHeights.CORAL_L3.getHeight();
        double a3 = PivotPresetAngles.CORAL_L3.getAngle();
        double l4 = PresetHeights.CORAL_L4.getHeight();
        double a4 = PivotPresetAngles.CORAL_L4.getAngle();

        double up = Superstructure.directTransferHeight(l3, a3, l4, a4);
        double down = Superstructure.directTransferHeight(l4, a4, l3, a3);
        assertFalse(Double.isNaN(up), "L3 -> L4 has an overlap to turn in");
        assertFalse(Double.isNaN(down), "L4 -> L3 has an overlap to turn in");
        // Never turns above the ceiling the robot itself imposed: the table
        // is optimistic at the top of travel (L4 caught the top bar there).
        assertTrue(up <= SuperstructureConstants.L4_PRE_TOP_HEIGHT, "turn height under the pre-top ceiling");
        assertTrue(down <= SuperstructureConstants.L4_PRE_TOP_HEIGHT, "turn height under the pre-top ceiling");
        // The turn is on the way, not a detour: it is between the two poses.
        assertTrue(up >= l3 && up <= l4, "the L3 -> L4 turn happens during the climb");
        assertTrue(down <= l4 && down >= l3, "the L4 -> L3 turn happens during the descent");

        // Every leg of both transfers, with the turn itself at that height.
        double[][] legs = {{l3, a3, up, l4, a4}, {l4, a4, down, l3, a3}};
        for (double[] leg : legs) {
            assertTrue(Superstructure.elevatorPathClear(leg[0], leg[2], leg[1]), "travel to the turn height");
            assertTrue(Superstructure.pivotPathClear(leg[1], leg[4], leg[2]), "the turn itself");
            assertTrue(Superstructure.elevatorPathClear(leg[2], leg[3], leg[4]), "carry on to the target");
            assertTrue(Superstructure.poseClear(leg[2], leg[1]), "turn height clear at the start angle");
            assertTrue(Superstructure.poseClear(leg[2], leg[4]), "turn height clear at the target angle");
        }
        // The arm only turns the few degrees between the two poses, never
        // out to RAISE: that is the whole point of the shortcut.
        assertTrue(Math.abs(a4 - a3) < SuperstructureConstants.BAND_PASS_MIN_ANGLE - a3,
            "the transfer rotation is the gap between the poses, not a RAISE excursion");

        // Nothing that crosses between the low box and the upper corridor:
        // the middle-stage top tube sits between them at every low angle.
        double l2 = PresetHeights.CORAL_L2.getHeight();
        double a2 = PivotPresetAngles.CORAL_L2.getAngle();
        double aBase = PivotPresetAngles.BASE.getAngle();
        double[][] crossing = {
            {BASE, aBase, l3, a3}, {BASE, aBase, l4, a4},
            {l2, a2, l3, a3}, {l2, a2, l4, a4},
            {l3, a3, BASE, aBase}, {l4, a4, BASE, aBase},
            {l3, a3, l2, a2}, {l4, a4, l2, a2},
        };
        for (double[] pair : crossing) {
            assertTrue(Double.isNaN(Superstructure.directTransferHeight(pair[0], pair[1], pair[2], pair[3])),
                "no overlap between the low box and the upper corridor: " + pair[1] + " deg -> " + pair[3] + " deg");
        }
        // Poses at or past a travel angle are excluded outright - there is no
        // excursion in those plans to remove.
        double algaeHigh = PresetHeights.ALGAE_HIGH_INTAKE.getHeight();
        double aAlgae = PivotPresetAngles.ALGAE_INTAKE.getAngle();
        assertTrue(Double.isNaN(Superstructure.directTransferHeight(l4, a4, BASE, PivotPresetAngles.CORAL_L1.getAngle())));
        assertTrue(Double.isNaN(Superstructure.directTransferHeight(l3, a3, algaeHigh, aAlgae)));
        assertTrue(Double.isNaN(Superstructure.directTransferHeight(algaeHigh, aAlgae, l4, a4)));
        assertTrue(Double.isNaN(Superstructure.directTransferHeight(PresetHeights.ALGAE_SCORE.getHeight(),
            PivotPresetAngles.ALGAE_SCORE.getAngle(), l3, a3)));

        // L4 keeps clear of the top of travel and of its own band ceiling,
        // so the carriage is not holding a height it has to fight for.
        assertTrue(l4 <= ElevatorConstants.ELEVATOR_MAX_POSITION - 1.0, "L4 stays an inch off the forward soft limit");
        assertTrue(Superstructure.poseClear(l4 + SuperstructureConstants.RATCHET_MARGIN, a4),
            "L4 is inside its corridor, not sitting on the ceiling");
    }
}
