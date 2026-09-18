package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * What the Superstructure needs from the elevator. It exists so the real
 * coordinated-motion commands can be run in unit tests against a simulated
 * carriage (SuperstructureSequenceSimTest): the vendor simulators advance on
 * wall-clock time, so the real subsystems cannot be stepped deterministically.
 * Heights in inches (preset frame), velocity in inches per second.
 */
public interface CarriageAxis extends Subsystem {
    double getCurrentPosition();

    double getVelocity();

    /** Commands a height at full pace; the controller profiles to it and holds it. */
    default void setPosition(double heightInches) {
        setPosition(heightInches, Pace.FULL);
    }

    /** Commands a height at the given pace (see {@link Pace}). */
    void setPosition(double heightInches, Pace pace);

    boolean isAtTargetPosition();

    /** Profile acceleration in effect at the current pace, in/s^2 (the planner sizes braking distances from it). */
    double maxAcceleration();

    /** Profile acceleration the given pace runs at, in/s^2. */
    double maxAcceleration(Pace pace);

    boolean isInManualMode();

    void manualControl(double stick);

    void holdCurrentPosition();
}
