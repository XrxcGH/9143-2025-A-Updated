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
    void presetPosesAreInsideExceptTheKnownTightL3() {
        assertTrue(Superstructure.poseClear(BASE, PivotPresetAngles.BASE.getAngle()));
        assertTrue(Superstructure.poseClear(BASE, PivotPresetAngles.CORAL_L1.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L2.getHeight(), PivotPresetAngles.CORAL_L2.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.CORAL_L4.getHeight(), PivotPresetAngles.CORAL_L4.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.ALGAE_LOW_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.ALGAE_HIGH_INTAKE.getHeight(), PivotPresetAngles.ALGAE_INTAKE.getAngle()));
        assertTrue(Superstructure.poseClear(PresetHeights.ALGAE_SCORE.getHeight(), PivotPresetAngles.ALGAE_SCORE.getAngle()));
        // RAISE is the travel angle: clear from the base to the top
        assertTrue(Superstructure.elevatorPathClear(BASE, ElevatorConstants.ELEVATOR_MAX_POSITION,
            PivotPresetAngles.RAISE.getAngle()));
        // The operator's L3 pose is the documented exception: the model puts
        // it inside a band, and the startup audit must say so.
        String audit = Superstructure.presetAuditMessage();
        assertTrue(audit.contains("L3"), "audit names the tight L3 pose: " + audit);
        assertFalse(audit.contains("L4"), "L4 must be clear: " + audit);
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
        // L3 return lift window: 25 deg up to RAISE clear at the lift height
        assertTrue(Superstructure.pivotPathClear(25.0, PivotPresetAngles.RAISE.getAngle(),
            SuperstructureConstants.MID_POSE_RETURN_LIFT_HEIGHT));
        // Algae: the stage angle is clear from the base; the intake angle needs the minimum height
        assertTrue(Superstructure.elevatorPathClear(BASE, ElevatorConstants.ELEVATOR_MAX_POSITION, SuperstructureConstants.HIGH_ANGLE_STAGE));
        assertTrue(Superstructure.elevatorPathClear(SuperstructureConstants.HIGH_ANGLE_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_POSITION,
            PivotPresetAngles.ALGAE_INTAKE.getAngle()));
    }
}
