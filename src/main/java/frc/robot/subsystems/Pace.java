package frc.robot.subsystems;

/**
 * How fast the carriage runs a move, as a fraction of its tuned profile
 * (cruise velocity AND acceleration, so a move simply takes 1 / scale as
 * long). The Superstructure picks a pace per move so that the carriage and
 * the arm travel TOGETHER: the carriage is quicker than the arm on almost
 * every move, and at full pace it reaches each clearance limit before the
 * arm has opened it, brakes, and is released again - sequential and jerky.
 * Paced to the arm it never has to brake.
 *
 * Each pace is one closed-loop slot on the Spark MAX (its MAXMotion limits
 * are per slot), so changing pace costs nothing: it is part of the setpoint.
 */
public enum Pace {
    FULL(1.0),
    BRISK(0.8),
    EASY(0.6),
    SLOW(0.4);

    public final double scale;

    Pace(double scale) {
        this.scale = scale;
    }
}
