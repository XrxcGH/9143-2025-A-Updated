package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Haptic feedback for one controller. It is a subsystem only so that rumble
 * commands REQUIRE it: a new cue pre-empts the one in progress, and the
 * interrupted command's end() is what switches the motor off, so a cue can
 * never be left buzzing. None of these run while disabled, which means the
 * scheduler cancels them (and end() zeroes the rumble) the moment the robot
 * disables.
 */
public class Rumble extends SubsystemBase {
    private final GenericHID hid;

    public Rumble(CommandXboxController controller) {
        hid = controller.getHID();
    }

    private void set(double strength) {
        hid.setRumble(RumbleType.kBothRumble, strength);
    }

    /** One buzz of the given strength (0-1) and length. */
    public Command pulse(double strength, double seconds) {
        return runEnd(() -> set(strength), () -> set(0.0)).withTimeout(seconds);
    }

    /** {@code count} short buzzes - a pattern the hands can tell from a single one. */
    public Command pulses(int count, double strength, double onSeconds, double offSeconds) {
        Command[] steps = new Command[count * 2 - 1];
        for (int i = 0; i < count; i++) {
            steps[2 * i] = pulse(strength, onSeconds);
            if (i < count - 1) {
                // idle() holds the requirement so nothing sneaks in between buzzes
                steps[2 * i + 1] = idle().withTimeout(offSeconds);
            }
        }
        return Commands.sequence(steps);
    }

    /** Buzzes for as long as the command is scheduled (bind with whileTrue). */
    public Command whileActive(double strength) {
        return runEnd(() -> set(strength), () -> set(0.0));
    }
}
