package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * What the Superstructure needs from the elevator. It exists so the REAL
 * coordinated-motion commands can be run in unit tests against a simulated
 * carriage (SuperstructureSequenceSimTest): the vendor simulators advance on
 * wall-clock time, so the real subsystems cannot be stepped deterministically.
 * Heights in inches (preset frame), velocity in inches per second.
 */
public interface CarriageAxis extends Subsystem {
    double getCurrentPosition();

    double getVelocity();

    /** Commands a height; the controller profiles to it and holds it. */
    void setPosition(double heightInches);

    boolean isAtTargetPosition();

    /** Profile acceleration in effect, in/s^2 (the planner sizes braking distances from it). */
    double maxAcceleration();

    boolean isInManualMode();

    void manualControl(double stick);

    void holdCurrentPosition();
}
