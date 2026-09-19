package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Application entry point. WPILib calls main() when the program starts on the
 * roboRIO (or in simulation) and hands control to the {@link Robot} class.
 *
 * Do not add any robot initialization here - anything that needs to run at
 * startup belongs in the Robot or RobotContainer constructors so it also
 * works correctly in simulation and unit tests.
 */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
