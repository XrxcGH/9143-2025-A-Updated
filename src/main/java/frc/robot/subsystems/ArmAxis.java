package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * What the Superstructure needs from the CorAl (pivot + rollers) - see
 * {@link CarriageAxis} for why this is an interface. Angles in degrees as
 * MEASURED on the real arm (the through bore), velocity in degrees per second.
 */
public interface ArmAxis extends Subsystem {
    double getPivotAngle();

    double getPivotVelocity();

    /** Commands an angle; the controller profiles to it and holds it. */
    void setPivotAngle(double angleDegrees);

    /**
     * Runs the arm's profile at a fraction of its tuned speed (the whole
     * profile is time-scaled: cruise x s, acceleration x s^2, jerk x s^3), so
     * a sweep that would finish long before the carriage can take its time.
     */
    void setProfileScale(double scale);

    /** The angle last commanded. */
    double getTargetAngle();

    boolean isAtTargetAngle();

    /** Profile acceleration in effect, deg/s^2. */
    double maxAcceleration();

    void manualPivotControl(double stick);

    void setIntakeSpeed(double dutyCycle);

    /** Runs the rollers to RELEASE a piece: as setIntakeSpeed, but never stopped by the arrival auto-stop. */
    void ejectRollers(double dutyCycle);

    void stopIntake();

    boolean isGamePieceDetected();
}
