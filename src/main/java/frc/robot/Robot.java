package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.util.Elastic;
import frc.robot.util.Tunables;

/**
 * The main robot class, called by WPILib at the appropriate times for each
 * robot mode (disabled, autonomous, teleop, test).
 *
 * This project is command-based: almost all robot behavior lives in the
 * subsystems and the command bindings configured in {@link RobotContainer}.
 * This class owns the mode lifecycle - scheduling the selected autonomous
 * command, cancelling it when teleop starts, running the command scheduler
 * every loop - plus the Elastic dashboard plumbing: it serves the layout
 * file to the dashboard, pushes live data every loop, and switches the
 * dashboard to the matching tab whenever the robot changes modes.
 *
 * Logging runs through AdvantageKit ({@link LoggedRobot}): DriverStation
 * data, joysticks, and console output are captured automatically, and every
 * Logger.recordOutput() call (see Dashboard) lands in a .wpilog file on the
 * roboRIO (open in AdvantageScope) and streams live over RLOG. CTRE's
 * SignalLogger (.hoot files, started in Telemetry) runs alongside for
 * Phoenix signals and SysId.
 */
public class Robot extends LoggedRobot {
	/** The autonomous command selected on the dashboard, scheduled in autonomousInit(). */
	private Command m_autonomousCommand;

	/** Container that owns all subsystems and controller bindings. */
	private final RobotContainer m_robotContainer;

	public Robot() {
		// ---- AdvantageKit logger ----
		// Metadata shows up in AdvantageScope's metadata tab for every log.
		Logger.recordMetadata("ProjectName", "9143-2025-A");
		Logger.recordMetadata("Robot", "A (competition)");
		// .wpilog files: to a USB stick (/U/logs) when one is plugged into
		// the roboRIO, otherwise /home/lvuser/logs; in simulation, ./logs.
		Logger.addDataReceiver(new WPILOGWriter());
		// Live stream for AdvantageScope's "Connect to Robot" (RLOG). Port
		// 5810 because the Elastic layout WebServer already owns 5800; both
		// are inside the field-legal 5800-5810 range.
		Logger.addDataReceiver(new RLOGServer(5810));
		Logger.start();
		DriverStation.silenceJoystickConnectionWarning(true);

		// Serve the deploy directory over HTTP (port 5800). Elastic uses this
		// to fetch deploy/elastic-layout.json via File -> "Load Layout From
		// Robot", so every drive station computer gets the same dashboard.
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

		// Seed the dashboard-editable tunables (vision distances, elevator
		// calibration/gains, handoff timing) with their Constants defaults
		// if not already stored on the roboRIO - BEFORE the subsystems are
		// built, since the Elevator configures its controllers from them.
		Tunables.init();

		m_robotContainer = new RobotContainer();

		// Warm up the PathPlanner path-following code (trajectory generation,
		// JSON parsing, JIT compilation) while the robot is sitting disabled.
		// Without this, the FIRST path of autonomous starts with a noticeable
		// delay/stutter, which shifts the whole routine.
		CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
	}

	/**
	 * Runs every 20 ms regardless of mode. The CommandScheduler poll is what
	 * makes the entire command-based framework work: it runs subsystem
	 * periodic() methods, polls triggers, and executes scheduled commands.
	 */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		// Publish all Elastic dashboard data (field pose, match time,
		// subsystem status, alerts) once per loop.
		m_robotContainer.updateDashboard();
	}

	@Override
	public void disabledInit() {
		// Stop all mechanism outputs when the robot is disabled
		m_robotContainer.disabledInit();

		// Show the pre/post-match checklist tab while disabled
		Elastic.selectTab("Setup");
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {
		// The code isn't fresh anymore! - flaco
		RobotContainer.freshCode = false; 
	}

	/** Schedules the autonomous routine selected in the dashboard's auto chooser. */
	@Override
	public void autonomousInit() {
		Elastic.selectTab("Autonomous");

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	/** Cancels any still-running autonomous command so drivers get control immediately. */
	@Override
	public void teleopInit() {
		Elastic.selectTab("Teleop");

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	/** Test mode: clear everything so test routines start from a clean slate. */
	@Override
	public void testInit() {
		Elastic.selectTab("Testing");

		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}

	@Override
	public void simulationPeriodic() {}
}
