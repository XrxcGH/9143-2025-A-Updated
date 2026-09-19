package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;

/**
 * How fast the carriage runs a move, as a fraction of its tuned profile
 * (cruise velocity and acceleration, so a move takes 1 / scale as
 * long). The Superstructure picks a pace per move so that the carriage and
 * the arm travel together: the carriage is quicker than the arm on almost
 * every move, and at full pace it reaches each clearance limit before the
 * arm has opened it, brakes, and is released again - sequential and jerky.
 * Paced to the arm it never has to brake.
 *
 * Each pace is one closed-loop slot on the Spark MAX (its MAXMotion limits
 * are per slot), so changing pace costs nothing: it is part of the setpoint.
 * The BRISK, EASY and SLOW scales are ElevatorConstants.PACE_*.
 */
public enum Pace {
    // 1.0 by definition, not a setting: FULL is the tuned profile itself, and
    // Elevator.cruiseVelocity() and the kG-from-cruise sampling assume it.
    // To run every move faster or slower, change the tuned profile (the
    // "Elevator - Cruise Velocity" / "Max Acceleration" tunables) instead.
    FULL(1.0),
    BRISK(ElevatorConstants.PACE_BRISK_SCALE),
    EASY(ElevatorConstants.PACE_EASY_SCALE),
    SLOW(ElevatorConstants.PACE_SLOW_SCALE);

    public final double scale;

    Pace(double scale) {
        this.scale = scale;
    }
}
