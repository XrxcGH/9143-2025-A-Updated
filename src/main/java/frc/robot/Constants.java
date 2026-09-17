package frc.robot;

/**
 * Robot-wide numerical and boolean constants, grouped by subsystem.
 * This class should not be used for any other purpose - all constants are
 * declared globally (public static) and nothing in here is functional code.
 *
 * Conventions used throughout this file:
 *  - Every value carries its unit in the comment (inches, degrees, amps, ...)
 *  - Phoenix 6 (TalonFX) closed-loop gains are voltage-based; Spark MAX PID
 *    gains are duty-cycle-based with voltage feedforward, as noted per section
 *  - Values marked TUNE are safe starting points for on-robot testing;
 *    values marked VERIFY are physical measurements that must be confirmed
 *    before the gains on top of them mean anything
 *
 * ============================== CAN ID MAP ==============================
 *   2       Pigeon 2 IMU                     (rio bus)
 *   5       CANdle LED controller
 *   11/12/13  Back Left  swerve drive / steer / CANcoder
 *   21/22/23  Front Left swerve drive / steer / CANcoder
 *   31/32/33  Front Right swerve drive / steer / CANcoder
 *   41/42/43  Back Right swerve drive / steer / CANcoder
 *   58/59   Elevator left (leader) / right (follower) Spark MAX
 *   60/61   CorAl pivot / intake TalonFX (Kraken X60)
 *   62      CANrange game piece sensor (VERIFY - see note at the constant)
 *
 *   (63 is free - previously reserved for the removed AlLow mechanism)
 *
 * ============================== DIO MAP =================================
 *   0       CorAl through bore encoder (absolute, duty cycle)
 * ========================================================================
 *
 * Swerve drivetrain constants live separately in generated/TunerConstants.java
 * because that file follows CTRE Tuner X's generated format.
 */
public final class Constants {

	/**
	 * Constants for the Elevator subsystem.
	 * Hardware: 2x NEO on Spark MAX, each through a 45:1 MAXPlanetary
	 * (5:1 x 3:1 x 3:1) to the winch drum. Right motor follows the left.
	 */
	public static final class ElevatorConstants {
		// --- CAN IDs ---
		public static final int ELEVATOR_LEFT_ID = 58;  // Left Spark MAX (leader)
		public static final int ELEVATOR_RIGHT_ID = 59; // Right Spark MAX (follower)

		// --- Motor Inversion ---
		public static final boolean ELEVATOR_LEFT_INVERTED = false;      // True if positive output should be flipped (positive must move the carriage UP)
		public static final boolean ELEVATOR_RIGHT_OPPOSES_LEFT = false; // True if the right motor spins opposite the left

		// --- Current Limits (amps) ---
		public static final int ELEVATOR_CURRENT_LIMIT = 50; // Spark MAX smart current limit per NEO

		// --- Mechanism Gearing ---
		// Power path: NEO -> 45:1 MAXPlanetary (5:1 x 3:1 x 3:1 cartridges)
		// -> 90-degree gearbox (1:1) -> 1/2" hex shaft -> 22T #25 sprockets
		// with chain runs at the top and bottom of the elevator.
		public static final double ELEVATOR_GEAR_RATIO = 5.0 * 3.0 * 3.0;

		// Chain advance (inches) per SPROCKET rotation. #25 chain has a
		// 0.25" pitch, so a 22T sprocket advances the chain exactly
		// 22 x 0.25 = 5.5 inches per rotation.
		public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE = 22 * 0.25;

		// Cascade rigging: the chain lifts the middle stage, and the cascade
		// run doubles that motion at the carriage - the carriage travels 2x
		// the chain advance.
		public static final double ELEVATOR_CASCADE_RATIO = 2.0;

		// Carriage travel (inches) per MOTOR rotation:
		// 5.5 x 2 / 45 = ~0.244. Applied as the Spark MAX position
		// conversion factor so all heights are in inches.
		public static final double ELEVATOR_INCHES_PER_ROTATION =
			ELEVATOR_SPROCKET_CIRCUMFERENCE * ELEVATOR_CASCADE_RATIO / ELEVATOR_GEAR_RATIO;

		// --- MAXMotion Profile (inches, seconds) ---
		// NEO free speed 5676 RPM = 94.6 rot/s -> 94.6 x 0.244 = ~23 in/s
		// theoretical top speed. Cruise stays well below that so the profile
		// remains achievable under load.
		// FIRST-POWER-ON VALUES: deliberately gentle (full 53 in of travel in
		// ~7 s, cruise reached in 0.33 s) so nothing slams while the gains are
		// being tuned. The profile always decelerates INTO the setpoint, so the
		// carriage settles rather than hitting the target. Once holding and
		// tracking are clean, raise toward 16 in/s and 60 in/s^2.
		public static final double ELEVATOR_MAX_VELOCITY = 8.0;      // Cruise velocity (in/s)
		public static final double ELEVATOR_MAX_ACCELERATION = 24.0; // Acceleration (in/s^2)

		// --- Closed-Loop PID Gains (Spark MAX slot 0; error in inches) ---
		// TUNE - first power-on procedure:
		//   1. Sanity-check the height reading against a tape measure (the
		//      conversion is derived from real gearing, but confirm the
		//      90-degree gearbox is 1:1 and rigging doesn't multiply travel).
		//   2. Raise kG until the carriage just holds its height at rest.
		//   3. Command a preset and raise kP until tracking is crisp;
		//      add kD only if it oscillates.
		// kP sanity: 0.4 duty/inch is ~0.1 duty per motor rotation (1 in =
		// 4.1 rotations), which is REV's own MAXMotion starting gain scaled to
		// this mechanism's units - the loop only has to track the profiled
		// setpoint (kV/kG carry the motion), so this is firm without ringing.
		// Symptoms: sluggish settle -> raise kP; buzz/hunt at rest -> lower.
		public static final double ELEVATOR_kP = 0.4; // Duty cycle per inch of position error
		public static final double ELEVATOR_kI = 0.0; // Leave 0 - kG handles gravity sag
		public static final double ELEVATOR_kD = 0.0; // Duty cycle per in/s of error derivative

		// --- On-Controller Feedforward (volts; REVLib FeedForwardConfig) ---
		public static final double ELEVATOR_kS = 0.0;  // Volts to overcome static friction (TUNE: raise until motion starts)
		public static final double ELEVATOR_kV = 0.52; // Volts per in/s of profile velocity (12 V / 23 in/s free speed)
		public static final double ELEVATOR_kG = 0.35; // Volts to hold the carriage against gravity (TUNE step 2)

		// --- Voltage Compensation ---
		public static final double ELEVATOR_NOMINAL_VOLTAGE = 12.0; // Volts; keeps response consistent as the battery sags

		// --- Position Limits (inches) ---
		// All heights are measured from the bottom of the carriage 2x1 to
		// the top of the base stage 2x1. The elevator is zeroed at its base
		// position on initialization, so the base IS zero; there is no
		// travel below it.
		public static final double ELEVATOR_MIN_POSITION = 0.0;  // Reverse soft limit
		public static final double ELEVATOR_MAX_POSITION = 53.0; // Forward soft limit

		// --- Tolerances (inches) ---
		public static final double ELEVATOR_ALLOWED_ERROR = 0.1; // "At target" threshold

		// --- Manual Control (unitless stick values) ---
		public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
		public static final double ELEVATOR_MANUAL_SPEED_LIMIT = 0.25;     // Max duty cycle in manual mode

		// --- Preset Heights (inches; bottom of carriage 2x1 to top of base stage 2x1) ---
		public enum PresetHeights {
			BASE(0.0),
			CORAL_L1(0.0),
			CORAL_L2(12.0),
			CORAL_L3(29.0),
			CORAL_L4(52.5),
			ALGAE_LOW_INTAKE(20.5),
			ALGAE_HIGH_INTAKE(37.5),
			ALGAE_SCORE(52.5);

			private final double height;

			PresetHeights(double height) {
				this.height = height;
			}

			public double getHeight() {
				return height;
			}
		}
	}

	/**
	 * Constants for the CorAl (Coral and Algae) subsystem.
	 * Hardware: Kraken X60 pivot + Kraken X60 intake rollers, REV Through
	 * Bore absolute encoder on the pivot, CANrange for game piece detection.
	 */
	public static final class CorAlConstants {
		// --- CAN IDs ---
		public static final int CORAL_PIVOT_MOTOR_ID = 60;  // Pivot TalonFX
		public static final int CORAL_INTAKE_MOTOR_ID = 61; // Intake TalonFX
		// VERIFY: this was previously documented/configured as 64, but Phoenix
		// device IDs only go up to 62 - constructing a device with ID 64
		// throws and crashes robot code on boot (caught by RobotContainerTest).
		// Set the CANrange to 62 in Tuner X, or change this to whatever ID it
		// actually has.
		public static final int CANRANGE_SENSOR_ID = 62;    // CANrange game piece sensor

		// --- DIO Ports ---
		public static final int THROUGH_BORE_DIO_PORT = 0; // Through bore encoder input

		// --- Motor Inversion ---
		public static final boolean CORAL_PIVOT_MOTOR_INVERTED = false; // True if positive output should be flipped (positive must raise the arm)
		public static final boolean CORAL_INTAKE_MOTOR_INVERTED = true; // True if positive output should be flipped (positive must intake coral)

		// --- Current Limits (amps) ---
		public static final int CORAL_PIVOT_CURRENT_LIMIT = 30;  // Pivot supply current limit (breaker protection)
		// Stator limit caps the pivot's TORQUE (a Kraken through 65:1 can
		// otherwise exert hundreds of N*m against a hard stop). 60 A is far
		// more than the arm needs to move, but well under a damaging slam.
		public static final int CORAL_PIVOT_STATOR_CURRENT_LIMIT = 60;
		public static final int CORAL_INTAKE_CURRENT_LIMIT = 20; // Intake supply current limit

		// --- Mechanism Gearing ---
		// Power path: Kraken X60 -> 10T pinion : 58T gear -> (same shaft)
		// 18T : 58T gear -> (same shaft) 12T #25 sprocket : 42T #25 sprocket
		// on the pivot. Total: (58/10) x (58/18) x (42/12) = ~65.41:1.
		public static final double CORAL_PIVOT_GEAR_RATIO =
			(58.0 / 10.0) * (58.0 / 18.0) * (42.0 / 12.0);

		// --- Motion Magic Profile (degrees, seconds; converted to mechanism rotations in the subsystem) ---
		// Conservative for first tests: the common 90-degree BASE<->RAISE
		// swing takes ~1.5 s, full 160-degree travel ~2.3 s (90 deg/s =
		// 16.4 rotor rps through 65.41:1 - well within a Kraken).
		// TUNE: raise once gains feel solid.
		// FIRST-POWER-ON VALUES: deliberately gentle (full 160 deg sweep in
		// ~3 s). 60 deg/s is only 11 rotor rps through 65.41:1 - a tenth of a
		// Kraken's free speed - and the jerk limit rounds the profile corners
		// so the arm eases into and out of every move instead of snapping.
		// Once tracking is clean, raise toward 120 deg/s and 240 deg/s^2.
		public static final double CORAL_PIVOT_MAX_VELOCITY = 60.0;      // Cruise velocity (deg/s)
		public static final double CORAL_PIVOT_MAX_ACCELERATION = 120.0; // Acceleration (deg/s^2)
		public static final double CORAL_PIVOT_MAX_JERK = 1200.0;        // Jerk limit (deg/s^3); ~10x accel = ~0.1 s to reach full accel

		// --- Closed-Loop Gains (Phoenix 6 slot 0; voltage-based, error in mechanism rotations) ---
		// TUNE - starting points for first power-on:
		//   1. Sanity-check the angle reading against the through bore /
		//      a protractor (ratio is derived from real tooth counts).
		//   2. Command a preset; raise kP until tracking is crisp, add kD if
		//      it oscillates.
		//   3. kG only works if 0 degrees = arm horizontal (Arm_Cosine);
		//      otherwise leave it 0.
		// kP sanity: 40 V/rot is 0.11 V per degree - the 2-degree at-target
		// tolerance is worth only 0.22 V, so this cannot ring on its own; the
		// profile feedforwards below do the moving and kP just corrects error.
		// Symptoms: stops short of the angle -> raise kP; buzz at rest -> lower.
		public static final double CORAL_PIVOT_kP = 40.0; // Volts per rotation of position error (~0.11 V/deg)
		public static final double CORAL_PIVOT_kI = 0.0;  // Leave 0 - use kG for gravity
		public static final double CORAL_PIVOT_kD = 0.0;  // Volts per rot/s of error derivative
		public static final double CORAL_PIVOT_kG = 0.0;  // Volts to hold the arm horizontal (requires 0 deg = horizontal)

		// --- Profile Feedforward (volts) ---
		// Physics-based so Motion Magic tracks its profile instead of lagging
		// it and catching up (which is what shows up as overshoot/oscillation
		// at the end of a move). kV = 12 V / free speed at the mechanism
		// (100 rotor rps / 65.41 = 1.53 pivot rot/s). kS overcomes static
		// friction so the arm starts moving without kP winding up first.
		public static final double CORAL_PIVOT_kS = 0.2;  // Volts to overcome static friction (TUNE: raise until motion starts)
		public static final double CORAL_PIVOT_kV = 7.85; // Volts per pivot rot/s of profile velocity

		// --- Pivot Angle Limits (degrees) ---
		// The pivot is zeroed at its base position on initialization, so the
		// base IS zero; there is no travel below it.
		public static final double CORAL_PIVOT_MIN_ANGLE = 0.0;   // Reverse soft limit
		public static final double CORAL_PIVOT_MAX_ANGLE = 160.0; // Forward soft limit

		// --- Tolerances (degrees) ---
		public static final double CORAL_PIVOT_ALLOWED_ERROR = 2.0; // "At target" threshold

		// --- Manual Control (unitless stick values) ---
		public static final double CORAL_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
		public static final double CORAL_MANUAL_SPEED_LIMIT = 0.1;      // Max duty cycle in manual mode

		// --- Game Piece Detection ---
		public static final double GAME_PIECE_DETECTION_CONFIRMATION_TIME = 0.3; // Seconds of continuous detection to confirm
		public static final double GAME_PIECE_DETECTION_THRESHOLD = 0.1;         // CANrange proximity threshold (meters)

		// --- Roller Speeds (duty cycle, -1 to 1; positive = coral intake direction) ---
		public static final double CORAL_INTAKE_SPEED = 0.1;   // Intaking coral (auto-stops on detection)
		public static final double CORAL_SCORE_SPEED = 0.3;    // Ejecting coral
		public static final double ALGAE_INTAKE_SPEED = -0.1;  // Intaking algae (reverse direction)
		public static final double ALGAE_HOLD_SPEED = -0.025;  // Holding pressure on algae
		public static final double ALGAE_SCORE_SPEED = 0.5;    // Ejecting algae

		// --- Through Bore Encoder (degrees) ---
		// The REV Through Bore reports one full revolution (360 degrees)
		// across its duty cycle range. VERIFY the encoder is mounted 1:1 on
		// the pivot shaft; if it rides a geared/chained shaft instead, scale
		// this by that stage's ratio.
		public static final double THROUGH_BORE_DEGREES_PER_ROTATION = 360.0;
		public static final double THROUGH_BORE_ALLOWED_DISCREPANCY = 2.0; // Max motor-vs-through-bore disagreement before re-sync

		// --- Preset Angles (degrees) ---
		// RAISE (90) is also the safe travel angle: the mechanism clears the
		// elevator structure at every height while the arm is at or above it.
		public enum PivotPresetAngles {
			BASE(0.0),          // Coral intake position
			RAISE(90.0),        // Safe travel / algae hold position
			CORAL_L1(100.0),
			CORAL_L2(5.0),
			CORAL_L3(22.5),
			CORAL_L4(45.0),
			// NOTE: intentionally equal to the forward soft limit (the
			// mechanical max). The soft limit only cuts output PAST the
			// threshold and the arm approaches from below with gravity
			// pulling back, so settling exactly at it works; if the arm
			// hunts against the limit during tuning, drop this 1-2 degrees.
			ALGAE_INTAKE(160.0),
			ALGAE_SCORE(105.0),
			ALGAE_HOLD(90.0);

			private final double angle;

			PivotPresetAngles(double angle) {
				this.angle = angle;
			}

			public double getAngle() {
				return angle;
			}
		}
	}

	/**
	 * Constants for coordinated elevator + CorAl motion (the Superstructure).
	 *
	 * Measured mechanism contact points (arm angle, elevator height):
	 *   1. (0 deg, 10.75 in)  - CorAl hits a static part of the elevator.
	 *      -> the arm may only be fully lowered near the base.
	 *   2. (5 deg, 23 in)     - top of the CorAl hits the bottom of the top
	 *      second-stage 2x1 while passing. -> the elevator only travels with
	 *      the arm at the safe angle (90 deg) above the low box; the L2 pose
	 *      (12 in, 5 deg) sits below the contact zone, and the L3 pose
	 *      (29 in, 22.5 deg) is entered with the mid handoff overlap.
	 *   3. (90 -> 45 deg at 52.5 in) - bottom of the CorAl hits the top of
	 *      the top second-stage 2x1 if the arm rotates down in place at full
	 *      height. -> L4 overlaps the rotation with elevator motion: rise at
	 *      90 deg past the handoff height, then rotate toward 45 deg while
	 *      the elevator finishes rising (and the mirror image on the way
	 *      back down).
	 *
	 * The safe-motion regions derived from these points:
	 *   - Below LOW_TRAVEL_MAX_HEIGHT with the arm at/above
	 *     ARM_CLEAR_MIN_ANGLE, everything is clear: the elevator and arm may
	 *     move freely and simultaneously ("the low box").
	 *   - The arm may only be below ARM_CLEAR_MIN_ANGLE (i.e. fully tucked)
	 *     when the elevator is below ARM_TUCK_MAX_HEIGHT.
	 *   - Above LOW_TRAVEL_MAX_HEIGHT, the elevator only travels with the
	 *     arm at/above RAISE (90 deg), except during the L4 handoff overlap.
	 */
	public static final class SuperstructureConstants {
		// Max elevator height at which the arm may be fully lowered to 0 deg
		// (contact occurs at 10.75 in; 0.75 in margin).
		public static final double ARM_TUCK_MAX_HEIGHT = 10.0; // Inches

		// Min arm angle that clears the static elevator part at any height
		// (the L2/L3 scoring angle). VERIFY: derived from the 5-degree
		// scoring poses being contact-free outside the tube zone.
		public static final double ARM_CLEAR_MIN_ANGLE = 5.0; // Degrees

		// Max elevator height for travel with the arm at ARM_CLEAR_MIN_ANGLE:
		// the second-stage tube contact occurs at 23 in (2 in margin). Below
		// this height, any arm angle >= ARM_CLEAR_MIN_ANGLE is safe at any
		// elevator position, so motion inside this "low box" needs no
		// safe-angle excursion.
		public static final double LOW_TRAVEL_MAX_HEIGHT = 21.0; // Inches

		// --- Handoff Overlap (L3 / L4 approaches) ---
		// Above the low box the arm cannot rotate below 90 deg in place, so
		// its final rotation overlaps the end of the elevator's climb. The
		// heights where that rotation starts are NOT stored anywhere: they
		// are DERIVED at plan time from the two mechanisms' motion profiles
		// (Superstructure.handoffHeight), so retuning either the elevator or
		// the pivot profile can never leave the overlap out of sync.
		//
		// The one tunable per pose is the RELATIONSHIP between the two
		// arrivals: how many seconds after the elevator settles the arm
		// finishes its rotation (negative = the arm finishes early).
		//   L4: -0.30 s -> with the shipped profiles the rotation starts at
		//       ~40 in and completes ~0.3 s before the elevator settles at
		//       52.5 in (arm done near 50 in - check tube clearance THERE).
		//   L3: +1.15 s -> starts at ~25 in; most of the rotation happens as
		//       the elevator settles at 29 in, so the mechanism sweeps in
		//       behind the tube rather than into it. PREDICTED - first test
		//       at low speed with a hand on the disable switch.
		// Both are editable live ("Superstructure - L3/L4 Arm Arrival
		// Offset (s)"); the resolved heights show on the Testing tab.
		public static final double L4_ARM_ARRIVAL_OFFSET_SECONDS = -0.30;
		public static final double L3_ARM_ARRIVAL_OFFSET_SECONDS = 1.15;

		// Derived handoffs are clamped to start no lower than this far above
		// the low-box roof (never inside the tube contact zone) ...
		public static final double HANDOFF_MIN_ABOVE_LOW_BOX = 1.0; // Inches
		// ... and no later than this far below the target, so the state gate
		// always opens before the elevator settles.
		public static final double HANDOFF_MIN_BEFORE_TARGET = 0.5; // Inches

		// Extra settling margin used when checking "arm is at/above the safe
		// travel angle" (degrees).
		public static final double SAFE_ANGLE_TOLERANCE = 3.0;

		// Timeout on the FINAL settle wait of every planned move (seconds).
		// If a mechanism can't quite reach its at-target tolerance (sensor
		// disagreement, mechanical stall against a stop), the command ends
		// anyway - the latched closed-loop setpoints keep holding position,
		// so timing out is always safe. Intermediate SAFETY gates (handoff
		// heights, safe-angle waits) never time out: skipping one of those
		// could command a collision.
		public static final double SETTLE_TIMEOUT_SECONDS = 3.0;
	}

	/**
	 * Constants for the LED subsystem (CTRE CANdle).
	 * The CANdle drives its 8 onboard LEDs plus an attached LED strip; the
	 * onboard LEDs are indices 0-7 and the strip starts at index 8.
	 */
	public static final class LEDConstants {
		// --- CAN IDs ---
		public static final int CANDLE_ID = 5; // CANdle (rio CAN bus)

		// --- Strip Configuration ---
		public static final int LED_COUNT = 68;       // Total LEDs: 8 onboard + strip (VERIFY - set to actual strip length)
		public static final double BRIGHTNESS = 0.6;  // Global brightness scalar 0-1 (limits current draw on long strips)

		// --- Timing (seconds) ---
		public static final double ENDGAME_WARNING_TIME = 20.0; // Teleop time remaining when the endgame pattern starts
	}

	/**
	 * Constants for teleop driving.
	 */
	public static final class DriveConstants {
		// Fraction of the drivetrain's theoretical top speed (and top
		// rotation rate) the driver sticks command at full deflection. This
		// is the DEFAULT for the "Drive - Teleop Speed Scale" tunable, which
		// can be changed from the dashboard without a redeploy: 0.25 for
		// indoor testing, raise toward 1.0 for competition driving.
		public static final double TELEOP_SPEED_SCALE = 0.25;

		// Stick deadband as a fraction of the SCALED top speed (so it stays
		// 20% of stick travel at every speed scale).
		public static final double STICK_DEADBAND = 0.2;
	}

	/**
	 * Constants for autonomous path following (PathPlanner).
	 * These gains are the feedback half of path following: PathPlanner already
	 * feeds forward the velocities/accelerations from the path, and these PID
	 * controllers correct whatever error remains between the robot's estimated
	 * pose and the path. If they are too low the robot never pulls itself back
	 * onto the path and finishes several feet off; too high and it oscillates.
	 */
	public static final class AutoConstants {
		// --- Translation Controller (m/s of correction per meter of error) ---
		public static final double TRANSLATION_kP = 5.0; // TUNE - PathPlanner's recommended starting point
		public static final double TRANSLATION_kI = 0.0; // Leave 0 - integral winds up over long paths
		public static final double TRANSLATION_kD = 0.0;

		// --- Rotation Controller (rad/s of correction per radian of error) ---
		public static final double ROTATION_kP = 5.0; // TUNE - PathPlanner's recommended starting point
		public static final double ROTATION_kI = 0.0;
		public static final double ROTATION_kD = 0.0;
	}

	/**
	 * Constants for the Vision subsystem (3x Limelight) and AprilTag tracking.
	 *
	 * Tracking coordinate frame (Limelight camera space):
	 *   X = right of the camera (+ means the tag appears to the robot's right)
	 *   Y = down (unused for driving)
	 *   Z = forward out of the lens (distance to the tag)
	 */
	public static final class VisionConstants {
		// --- Limelight Names (NetworkTables: "limelight-" + name) ---
		public static final String[] LIMELIGHT_NAMES = {"funnel", "barge", "reef"};

		// --- Pipelines ---
		public static final int APRILTAG_PIPELINE = 0; // Pipeline index used for AprilTag detection

		// --- Thermal Management ---
		// Limelight "throttle": the camera processes one frame, then skips
		// this many. While the robot is DISABLED (pit, queue, between
		// periods - most of the camera's powered-on life) full-rate AprilTag
		// processing is wasted work that just heats the camera and spins the
		// fan; Limelight's guidance is 100-200. Full rate (0) is restored
		// the instant the robot enables, so tracking performance is
		// unaffected. At ~90 fps, 100 still yields ~1 solve/s while disabled,
		// plenty for the pre-match heading seed.
		public static final int DISABLED_THROTTLE = 100;
		public static final int ENABLED_THROTTLE = 0;

		// --- Camera Mounting ---
		// Facing sign per Limelight, same order as LIMELIGHT_NAMES:
		// +1 = camera faces the robot's FRONT, -1 = faces the REAR. The
		// tracker mirrors its drive commands for rear-facing cameras (used
		// when backing up to the coral station). VERIFY against the actual
		// camera mounting.
		public static final double[] LIMELIGHT_FACING_SIGNS = {
			-1.0, // funnel - rear-facing (watches the coral station behind the robot)
			1.0,  // barge  - front-facing
			1.0,  // reef   - front-facing
		};

		// --- Camera Poses in Robot Space ---
		// Where each lens sits on the robot. MegaTag uses this to convert what
		// a camera sees into where the ROBOT is, so an error here shifts every
		// fused pose from that camera. Pushed to the camera at startup (so it
		// lives in version control and survives a camera reset), but ONLY for
		// cameras marked measured - unmeasured cameras keep whatever their
		// web UI holds.
		//
		// Limelight robot-space convention (docs "3D Coordinate Systems"):
		// origin at the frame center projected to the floor; X+ forward,
		// Y+ toward the robot's RIGHT (note: opposite of WPILib's +Y = left),
		// Z+ up; meters and degrees. Pitch is entered as positive = lens
		// tilted UP, yaw as the heading of the lens (180 = rear-facing).
		// VERIFY the pitch/yaw signs once against the camera's web-UI 3D
		// preview (it mirrors these values) - if the preview shows the
		// camera pointing the wrong way, flip the sign here.
		public static final class CameraPose {
			public final double forwardMeters, sideMeters, upMeters;
			public final double rollDegrees, pitchDegrees, yawDegrees;
			/** False = placeholder; the pose is NOT pushed to the camera. */
			public final boolean measured;

			public CameraPose(double forwardMeters, double sideMeters, double upMeters,
					double rollDegrees, double pitchDegrees, double yawDegrees, boolean measured) {
				this.forwardMeters = forwardMeters;
				this.sideMeters = sideMeters;
				this.upMeters = upMeters;
				this.rollDegrees = rollDegrees;
				this.pitchDegrees = pitchDegrees;
				this.yawDegrees = yawDegrees;
				this.measured = measured;
			}
		}

		// Same order as LIMELIGHT_NAMES.
		public static final CameraPose[] LIMELIGHT_POSES = {
			// funnel (rear): lens 1.0 in inside the BACK edge of the 30 in
			// frame -> 14.0 in behind center; centered side-to-side;
			// 29.625 in above the floor; facing straight back and tilted
			// 50 deg above horizontal ("50 degrees north of west" in the
			// side view with west = rearward, north = up) so the coral
			// station tag stays in frame when the rear bumpers are flush.
			new CameraPose(-14.0 * 0.0254, 0.0, 29.625 * 0.0254, 0.0, 50.0, 180.0, true),
			// barge (front): PLACEHOLDER - measure and set measured = true
			new CameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
			// reef (front): PLACEHOLDER - measure and set measured = true
			new CameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
		};

		// --- Tag Classes (2025 Reefscape field) ---
		// Only reef and coral station tags are tracked; barge (4, 5, 14, 15)
		// and processor (3, 16) tags are intentionally left blank for now -
		// the tracker ignores them entirely.
		public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
		public static final int[] CORAL_STATION_TAGS = {1, 2, 12, 13};

		// --- Tracking Goal Distances (meters, camera-space Z) ---
		// All "flush" distances are what the camera READS in that condition,
		// not a field dimension. TUNE by physically placing the robot in the
		// goal position and copying the Vision/Distance dashboard value.
		public static final double REEF_FLUSH_DISTANCE = 0.45;    // Front bumpers flush with the reef base (L2-L4 + algae)
		public static final double L1_SCORE_DISTANCE = 1.0;       // Standoff for L1 so the CorAl can swing to 100 deg without hitting
		public static final double STATION_FLUSH_DISTANCE = 0.45; // Rear bumpers flush with the coral station wall (rear camera reading)

		// --- Reef Branch Alignment (meters, camera-space X) ---
		// Each reef face has two scoring branches, one either side of the
		// tag. Lateral offset from tag center to a branch center (~13 in
		// branch spacing / 2). TUNE on the field.
		public static final double REEF_BRANCH_OFFSET = 0.165;

		// --- Tracking Gains and Limits ---
		public static final class TrackingGains {
			public static final double DISTANCE_kP = 1.5;  // TUNE - m/s of drive per meter of position error
			public static final double ROTATION_kP = 0.06; // TUNE - rad/s of rotation per DEGREE of tx error

			public static final double POSITION_ERROR_DEADBAND = 0.05; // Meters; errors below this are treated as zero
			public static final double ROTATION_ERROR_DEADBAND = 1.0;  // Degrees of tx; errors below this are treated as zero

			public static final double MAX_LINEAR_VELOCITY = 2.0;  // m/s command clamp while tracking
			public static final double MAX_ANGULAR_VELOCITY = 1.0; // rad/s command clamp while tracking
		}
	}

	/**
	 * Constants for AdvantageKit / AdvantageScope logging output.
	 */
	public static final class LoggingConstants {
		// Indices into the Components3d / DesiredComponents3d Pose3d arrays
		// published for AdvantageScope's articulated 3D robot model. This
		// robot's three moving components, in the order the code publishes
		// them (must match the component order in
		// advantageScopeAssets/Robot_Leviathan/config.json / the glTF export):
		public static final int MIDDLE_STAGE_INDEX = 0; // Elevator middle stage (rises at half carriage speed)
		public static final int CARRIAGE_INDEX = 1;     // Elevator carriage
		public static final int ARM_INDEX = 2;          // CorAl arm
	}
}
