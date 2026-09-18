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

    /** The angle last commanded. */
    double getTargetAngle();

    boolean isAtTargetAngle();

    /** Profile acceleration in effect, deg/s^2. */
    double maxAcceleration();

    void manualPivotControl(double stick);

    void setIntakeSpeed(double dutyCycle);

    void stopIntake();

    boolean isGamePieceDetected();
}
