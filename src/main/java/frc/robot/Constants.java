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
 *   3       CANrange game piece sensor
 *   5       CANdle LED controller (not installed - LED code commented out)
 *   11/12/13  Back Left  swerve drive / steer / CANcoder
 *   21/22/23  Front Left swerve drive / steer / CANcoder
 *   31/32/33  Front Right swerve drive / steer / CANcoder
 *   41/42/43  Back Right swerve drive / steer / CANcoder
 *   58/59   Elevator left (leader) / right (follower) Spark MAX
 *   56/57   CorAl pivot / intake TalonFX (Kraken X60)
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
	 * Hardware: 2x NEO on Spark MAX, each through a 15:1 MAXPlanetary
	 * (5:1 x 3:1) to the sprocket shaft. Right motor follows the left.
	 */
	public static final class ElevatorConstants {
		// --- CAN IDs ---
		public static final int ELEVATOR_LEFT_ID = 58;  // Left Spark MAX (leader)
		public static final int ELEVATOR_RIGHT_ID = 59; // Right Spark MAX (follower)

		// --- Motor Inversion ---
		public static final boolean ELEVATOR_LEFT_INVERTED = false;      // True if positive output should be flipped (positive must move the carriage UP)
		public static final boolean ELEVATOR_RIGHT_OPPOSES_LEFT = true; // True if the right motor spins opposite the left

		// --- Current Limits (amps) ---
		public static final int ELEVATOR_CURRENT_LIMIT = 50; // Spark MAX smart current limit per NEO

		// --- Mechanism Gearing ---
		// Power path: NEO -> 15:1 MAXPlanetary (5:1 x 3:1 cartridges; the
		// second 3:1 was removed in Sept 2026 for speed) -> 90-degree gearbox
		// (1:1, REV-21-2120) -> 1/2" hex shaft -> 22T #25 sprockets with
		// chain runs at the top and bottom of the elevator.
		public static final double ELEVATOR_GEAR_RATIO = 5.0 * 3.0;

		// Chain advance (inches) per SPROCKET rotation. #25 chain has a
		// 0.25" pitch, so a 22T sprocket advances the chain exactly
		// 22 x 0.25 = 5.5 inches per rotation.
		public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE = 22 * 0.25;

		// Cascade rigging: the chain lifts the middle stage, and the cascade
		// run doubles that motion at the carriage - the carriage travels 2x
		// the chain advance.
		public static final double ELEVATOR_CASCADE_RATIO = 2.0;

		// Carriage travel (inches) per MOTOR rotation predicted by the
		// gearing alone: 5.5 x 2 / 15 = ~0.733.
		public static final double ELEVATOR_MODELED_INCHES_PER_ROTATION =
			ELEVATOR_SPROCKET_CIRCUMFERENCE * ELEVATOR_CASCADE_RATIO / ELEVATOR_GEAR_RATIO;

		// MEASURED correction on top of the model - the factory default of
		// the "Elevator - Travel Ratio" tunable (the live value is what the
		// Spark MAX gets; see Elevator.inchesPerRotation()).
		//
		// CONFIRMED 1.0 on the robot (Sept 2026). Five tape tests, measured
		// from the top of the base-stage 2x1 to the bottom of the carriage
		// 2x1, all fit ONE line: measured = model travel + ~0.875 in.
		//   commanded   ratio applied   model travel   tape
		//      2.00        1.0              2.00        ~2.75
		//      6.00        1.375            4.36         5.125
		//     20.00        1.17            17.03        ~17.9
		//     40.00        1.17            34.05        ~35
		// (the 8 in test, 6.375, ran during the follower-brake stutter and
		// is the one outlier). Between the 20 and 40 in tests the slope is
		// 1.005, so the gearing model is right to within 0.5%; the constant
		// is NOT a scale error but the height the carriage sits at on its
		// hard stop in that measurement frame - see ELEVATOR_ZERO_HEIGHT.
		// The two earlier "corrections" (1.375, then 1.17) were fits to that
		// offset and are gone.
		public static final double ELEVATOR_MEASURED_TRAVEL_RATIO = 1.0;

		// Default carriage travel (inches) per MOTOR rotation (~0.733): the
		// model times the default ratio. Anything that needs the LIVE value
		// asks the Elevator (inchesPerRotation()).
		public static final double ELEVATOR_INCHES_PER_ROTATION =
			ELEVATOR_MODELED_INCHES_PER_ROTATION * ELEVATOR_MEASURED_TRAVEL_RATIO;

		// --- MAXMotion Profile (inches, seconds) - tunable defaults ---
		// NEO free speed 5676 RPM = 94.6 rot/s -> 94.6 x 0.733 = ~69 in/s
		// theoretical top speed through 15:1. Cruise stays at ~72% of that
		// so the profile remains achievable under load: kV x 50 = ~8.7 V
		// plus kS/kG, leaving ~2.5 V for the position loop on a sagging
		// battery. Full 52 in of travel takes ~1.2 s (cruise reached in
		// 0.125 s over 3.1 in). 400 in/s^2 (~1 g) costs the motors only
		// ~0.6 V against the reflected inertia; it is a comfort limit on the
		// chain, the cascade cable and the arm riding on the carriage - and
		// the deceleration is what the staged L3/L4 sequences see when the
		// carriage stops at a station, so do not raise it casually.
		// The profile always decelerates INTO the setpoint, so the carriage
		// settles rather than hitting the target. Adjust on the Testing tab
		// ("Elevator - Cruise Velocity" / "Max Acceleration"). The
		// Superstructure's staged sequences are gated on MEASURED state, so
		// they stay safe at any speed - only their duration changes.
		public static final double ELEVATOR_MAX_VELOCITY = 50.0;      // Cruise velocity (in/s)
		public static final double ELEVATOR_MAX_ACCELERATION = 400.0; // Acceleration (in/s^2)
		// How far the carriage may stray from the MAXMotion profile before
		// the controller regenerates the profile from the current position
		// and velocity. NOT a settling tolerance. REV's starting point is
		// about one motor rotation; 0.3 in is ~0.4 rotation through 15:1.
		public static final double ELEVATOR_ALLOWED_PROFILE_ERROR = 0.3;

		// --- Closed-Loop Gains (Spark MAX slot 0; error in inches) - tunable defaults ---
		// kP, kS, the kV scale, kG, and the profile above are all editable on
		// the Testing tab and re-applied to both controllers the next time
		// the robot is disabled - no redeploy, no REV Hardware Client.
		// TUNE - first power-on procedure:
		//   1. Calibrate the height reading against a tape measure (README:
		//      "Calibrating the elevator height").
		//   2. Raise kG until the carriage just holds its height at rest.
		//   3. Command a preset and raise kP until tracking is crisp;
		//      add kD only if it oscillates.
		// kP sanity: REV's MAXMotion starting point is 0.01 duty per motor
		// ROTATION; through 15:1 one inch is only ~1.4 rotations, so that is
		// ~0.014 duty/in and 0.1 is ~7x it - the same multiple the 45:1
		// gearing ran happily at (0.3 duty/in there). The loop only tracks
		// the profiled setpoint (kV/kG carry the motion) and the NEO's
		// back-EMF damping keeps a P-only loop overdamped, so this is firm
		// without ringing; its job is to shrink the friction-induced rest
		// error (rest error = friction volts / (12 x kP) inches).
		// Symptoms: rests short of target -> raise kP or kS; buzz/hunt at
		// rest -> lower kP; overshoots -> lower the kV scale, then kS (REV).
		public static final double ELEVATOR_kP = 0.1; // Duty cycle per inch of position error
		public static final double ELEVATOR_kI = 0.0; // Leave 0 - kG handles gravity sag
		public static final double ELEVATOR_kD = 0.0; // Duty cycle per in/s of error derivative

		// --- On-Controller Feedforward (volts; REVLib FeedForwardConfig) ---
		// kS: REV's procedure - the largest voltage that does NOT move the
		// carriage (TUNE: raise until motion just starts, then back off).
		// A modest starting value breaks the stick-slip that a P-only loop
		// shows at the slow end of a profile (the carriage sticks, error
		// builds, it lurches free - "stutters and slows down"). Through
		// 15:1 this is a small torque, so it cannot cause a runaway.
		public static final double ELEVATOR_kS = 0.2;
		// kV is NOT a stored constant. It is the NEO back-EMF model,
		// 12 V / (free speed in in/s), and free speed in inches depends on
		// the travel ratio - so the Elevator derives it from the live ratio
		// (modelKv: ~0.17 V per in/s at 0.733 in/rot through 15:1). The kV
		// scale multiplies that model (REV: overshoot -> reduce kV).
		public static final double NEO_FREE_SPEED_RPM = 5676.0;
		public static final double ELEVATOR_kV_SCALE = 1.0;
		// kG: carriage + arm weight through the 2:1 cascade and 15:1 works
		// out to ~0.7 V per motor without help - but the CAD shows
		// constant-force springs on the carriage rigging, which carry an
		// unknown share of that. 0.6 is the no-help estimate less a little;
		// tune it down until the carriage just holds (TUNE step 2).
		public static final double ELEVATOR_kG = 0.6;

		/** NEO back-EMF velocity feedforward (volts per in/s) for a given carriage travel per motor rotation. */
		public static double modelKv(double inchesPerRotation) {
			return ELEVATOR_NOMINAL_VOLTAGE / (NEO_FREE_SPEED_RPM / 60.0 * inchesPerRotation);
		}

		// --- Voltage Compensation ---
		public static final double ELEVATOR_NOMINAL_VOLTAGE = 12.0; // Volts; keeps response consistent as the battery sags

		// --- Position Frame and Limits (inches) ---
		// All heights (presets, contact points, soft limits, dashboard) are
		// measured from the top of the base-stage 2x1 to the bottom of the
		// carriage 2x1, with the middle stage between them. On its hard stop
		// the carriage does NOT sit at 0 in that frame: five tape tests put
		// it ~0.875 in above the reference (see the travel-ratio note), and
		// the CAD (Leviathan STEP, everything on its hard stops) puts it at
		// exactly 1.000 in - the base stage's bottom cross tube tops out at
		// Y 5.875, the middle stage's 1 in tube sits on it, and the carriage
		// tube bottom is at 6.875. Tape at rest settles which. The encoder
		// is referenced TO this value, not zeroed, every time the carriage
		// is at its hard stop, so a commanded 20 in lands the tape at 20 in.
		// Default of the "Elevator - Height At Hard Stop" tunable. The
		// reverse soft limit is this height; presets of 0 clamp to it.
		public static final double ELEVATOR_ZERO_HEIGHT = 0.875;
		public static final double ELEVATOR_MAX_POSITION = 53.0; // Forward soft limit

		// --- Tolerances (inches) ---
		public static final double ELEVATOR_ALLOWED_ERROR = 0.1; // "At target" threshold
		// The encoder must read within this of the hard-stop height for a
		// calibration edit (travel ratio / hard-stop height) to be applied:
		// the carriage is then on its hard stop and is re-referenced there.
		public static final double ELEVATOR_AT_BASE_TOLERANCE = 0.5;
		// A reading this far BELOW the hard-stop height means the encoder was
		// referenced with the carriage raised and it has since dropped to
		// the hard stop - every commanded height would land that much high.
		// The Dashboard alerts to re-zero.
		public static final double ELEVATOR_BELOW_ZERO_ALERT = -0.25;

		// --- Manual Control (unitless stick values) ---
		public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
		public static final double ELEVATOR_MANUAL_SPEED_LIMIT = 0.25;     // Max duty cycle in manual mode

		// --- Preset Heights (inches; bottom of carriage 2x1 to top of base stage 2x1) ---
		public enum PresetHeights {
			BASE(0.0),
			CORAL_L1(0.0),
			CORAL_L2(12.0),
			CORAL_L3(30.5),
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
		public static final int CORAL_PIVOT_MOTOR_ID = 56;  // Pivot TalonFX
		public static final int CORAL_INTAKE_MOTOR_ID = 57; // Intake TalonFX
		// VERIFY: this was previously documented/configured as 64, but Phoenix
		// device IDs only go up to 62 - constructing a device with ID 64
		// throws and crashes robot code on boot (caught by RobotContainerTest).
		public static final int CANRANGE_SENSOR_ID = 3;    // CANrange game piece sensor

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
		// DEFAULTS for the live tunables "Pivot - Cruise Velocity" /
		// "Acceleration" / "Jerk" (Testing tab), re-applied to the TalonFX
		// the next time the robot is disabled.
		//
		// CHAIN-BACKLASH PROFILE (Sept 2026): the pivot chain currently has
		// slack and backlash, so at the end of a fast move the arm coasts
		// through the slop and the chain catches it - the whole assembly
		// gets thrown around. Motion Magic decelerates at the same rate it
		// accelerates, so the ACCELERATION sets how hard that catch is, and
		// the JERK limit sets how abruptly the deceleration begins. These
		// values keep the arm ~2x faster than the first-power-on profile
		// (90-degree swing ~1.2 s instead of 2.1 s, the 45-degree L4
		// rotation ~0.9 s) while easing into every stop:
		//   - 200 deg/s is 36% of the Kraken's free speed through 65.41:1
		//     (100 rotor rps / 65.41 = 1.53 rot/s = 550 deg/s): ~4.4 V of
		//     kV at cruise, plenty of headroom for kP.
		//   - 300 deg/s^2: 37% less peak force at the chain catch than the
		//     480 the mechanism can otherwise handle.
		//   - 2000 deg/s^3: ~0.15 s to reach full deceleration, so the stop
		//     ramps in instead of snapping.
		// WHEN THE CHAIN IS FIXED (no slop): 240 / 480 / 4800 tracked
		// cleanly and is the match-pace target - set it on the Testing tab,
		// then move these defaults. If a held coral slips during a swing,
		// lower the ACCELERATION first (it sets the peak tangential force),
		// then cruise.
		// The Superstructure handoff heights follow the live values - re-check
		// the Handoffs readout after a change.
		public static final double CORAL_PIVOT_MAX_VELOCITY = 200.0;     // Cruise velocity (deg/s)
		public static final double CORAL_PIVOT_MAX_ACCELERATION = 300.0; // Acceleration (deg/s^2)
		public static final double CORAL_PIVOT_MAX_JERK = 2000.0;        // Jerk limit (deg/s^3); ~0.15 s to reach full accel

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

		// --- Game Piece Detection (CANrange) ---
		// The CANrange's own proximity bit is "distance < threshold", with a
		// hysteresis band around it, and only when the return is strong
		// enough to count as a valid measurement. With the original 0.1 m
		// threshold and the 0.01 m default hysteresis the empty claw's own
		// structure sat right at the threshold, so the bit chattered and the
		// rising-only debounce latched it as a coral. The fixes: a hysteresis
		// band wide enough to sit clear of that background, a narrower
		// field of view so oblique structure is not in the beam, a health
		// check, and a debounce on BOTH edges. The threshold and hysteresis
		// are live tunables ("CorAl - Coral Detect ..."): read
		// CorAl/CANrange Distance on the Testing tab with the claw empty and
		// with a coral, and put the threshold halfway between.
		public static final double GAME_PIECE_DETECTION_CONFIRMATION_TIME = 0.3; // Seconds each edge must persist (both edges)
		public static final double GAME_PIECE_DETECTION_THRESHOLD = 0.08;        // Tunable default: detect below this (meters)
		public static final double GAME_PIECE_DETECTION_HYSTERESIS = 0.015;      // Tunable default: detect below threshold - this, release above threshold + this (meters)
		public static final double GAME_PIECE_MIN_SIGNAL_STRENGTH = 2500;        // Below this the measurement is invalid and cannot detect (CTRE default)
		public static final double GAME_PIECE_FOV_DEGREES = 13.5;                // Field of view, both axes (6.75 min, 27 max); narrow keeps claw structure out of the beam

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
		// 0 deg is the CAD's intake pose: the claw points up and ~33 deg past
		// vertical toward the REAR (the funnel); positive angles rotate it
		// forward. RAISE is the safe travel angle: per the CAD clearance map
		// (SuperstructureConstants) 100 deg is clear from ~4 in to the top,
		// while 90 deg is blocked above ~36.5 in by the middle-stage top
		// tube meeting the claw's lower rear.
		public enum PivotPresetAngles {
			BASE(0.0),          // Coral intake position
			RAISE(100.0),       // Safe travel / algae hold position
			CORAL_L1(100.0),
			// L2 scores at 12.5 rather than the geometric 5: the pivot chain
			// currently has backlash, so the arm sags a few degrees past
			// where the rotor thinks it is. At 12 in the CAD gives 10 deg a
			// full inch to the cross bar but only 0.5 in at 7.5; 12.5 keeps
			// the inch even 2.5 deg low. Revisit when the chain is fixed.
			CORAL_L2(12.5),
			// L3 was 22.5 at 29 in, a pose the CAD puts ~0.25 in from the
			// middle-stage top sprocket shaft. 25 deg at 30.5 in is the
			// nearest pose with a full inch (claw tip 0.7 in further forward
			// and 1.6 in higher); 27.5 deg there has 1.75 in if it ever rubs.
			CORAL_L3(25.0),
			// L4 was 45, then asked to be 35 "to bring the claw back more".
			// The CAD map says neither is reachable at 52.5 in: from ~25 to
			// ~85 deg the claw's lower rear meets the middle-stage top tube
			// and spring hardware at the top of travel (the bar L4 hit).
			// The top of travel is clear only at <= 22.5 deg; 20 gives
			// ~1.25 in of model clearance (15-20 all work if the coral
			// wants a different tilt - stay at or below 22.5).
			CORAL_L4(20.0),
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
	 * The free region of (elevator height, arm angle) was computed from the
	 * robot CAD (9143-2025-A-0000 Leviathan STEP): every CorAl part's true
	 * outline, swept about the pivot, against the base stage (static), the
	 * middle stage (which rises at HALF the carriage travel), the funnel,
	 * frame and bumpers, at 0.25 in resolution. The model reproduces both
	 * contacts measured on the robot - 0 deg at ~10.75 in (the cross bar
	 * under the top sprockets) and 5 deg at ~23 in (the middle-stage top
	 * tube) - within half an inch.
	 *
	 * What it shows (heights in the preset frame, base-stage top tube to
	 * carriage bottom tube):
	 *   - Tucked (< 8 deg): clear only below ~9 in - the static cross bar
	 *     under the top sprockets.
	 *   - Low box: with the arm anywhere from 8 to 65 deg the carriage may
	 *     rise to ~17.5-22 in; then the middle-stage top tube blocks a band
	 *     ("band A", roughly 18-30 in depending on angle) until the tube,
	 *     rising at half speed, has moved out of the claw's way.
	 *   - Above band A there is a high corridor for 20-65 deg whose CEILING
	 *     is where the claw's lower rear (motor bracket) meets the
	 *     middle-stage top tube and constant-force-spring hardware again
	 *     ("band B": ~41 in at 60 deg, ~46 in at 45 deg, ~51 in at 30 deg).
	 *     At the top of travel ONLY angles <= 22.5 deg (or >= ~97 deg) are
	 *     clear. That is the bar L4 was hitting; it is why the L4 pose is
	 *     20 deg and why the arm can never rotate down to L4 near the top -
	 *     the rotation happens at the 33 in "station" instead.
	 *   - 90 deg is NOT clear at every height: band B blocks it from
	 *     ~36.5 in up. 100 deg is clear from ~4 in to the top, so RAISE (the
	 *     safe travel / algae hold angle) is 100.
	 *   - Angles > 130 deg hit the bumper / brainpan below ~5-7.5 in.
	 *
	 * FREE_CORRIDORS is that map with a 1.0 in model margin (about 1.0-1.5 in
	 * real). The planner's staged sequences were simulated against the full
	 * model with the 15:1 elevator (50 in/s, 400 in/s^2) and the softened
	 * pivot profile, sampled every 10 ms: every transit clears by >= 1.0 in,
	 * also with the arm slowed to 150 deg/s (chain backlash). Every preset
	 * is inside a corridor: the L3 pose moved from (29 in, 22.5 deg), which
	 * the model put within ~0.25 in of the top sprocket shaft, to
	 * (30.5 in, 25 deg), the nearest pose with a full inch.
	 */
	public static final class SuperstructureConstants {
		// --- CAD free corridors ---
		// Rows: {angle max (exclusive), low corridor min, low corridor max,
		// high corridor min, high corridor max}; heights in inches, preset
		// frame; NaN = no separate high corridor at that angle. The last row
		// runs past the 160 deg soft limit so the limit itself is covered. A pose is
		// clear when its height lies in either corridor of its angle row.
		// (Superstructure.poseClear evaluates it; the Testing-tab single-
		// mechanism moves and a startup preset audit use it.)
		public static final double[][] FREE_CORRIDORS = {
			{  5.0,  0.0,   8.5,  43.0,  53.5},
			{ 10.0,  0.0,   9.0,  40.5,  53.5},
			{ 15.0,  0.0,  20.5,  38.5,  53.5},
			{ 20.0,  0.0,  22.0,  37.5,  53.5},
			{ 25.0,  0.0,  22.0,  35.5,  52.5},
			{ 30.0,  0.0,  22.0,  30.0,  51.0},
			{ 35.0,  0.0,  21.5,  29.5,  49.5},
			{ 40.0,  0.0,  20.5,  29.0,  48.0},
			{ 45.0,  0.0,  20.0,  28.0,  46.5},
			{ 50.0,  0.0,  19.0,  27.5,  45.0},
			{ 55.0,  0.0,  18.5,  26.5,  43.5},
			{ 60.0,  0.0,  18.0,  25.5,  42.0},
			{ 65.0,  0.0,  17.5,  24.5,  41.0},
			{ 70.0,  0.0,  17.5,  22.5,  40.0},
			{ 75.0,  0.0,  39.0,   Double.NaN,   Double.NaN},
			{ 80.0,  0.0,  38.0,   Double.NaN,   Double.NaN},
			{ 85.0,  0.0,  37.0,   Double.NaN,   Double.NaN},
			{ 90.0,  0.0,  36.5,   Double.NaN,   Double.NaN},
			{ 95.0,  0.0,  36.0,  53.0,  53.5},
			{100.0,  0.0,  36.5,  49.0,  53.5},
			{105.0,  0.0,  53.5,   Double.NaN,   Double.NaN},
			{110.0,  0.0,  53.5,   Double.NaN,   Double.NaN},
			{115.0,  0.0,  53.5,   Double.NaN,   Double.NaN},
			{120.0,  4.0,  53.5,   Double.NaN,   Double.NaN},
			{125.0,  4.0,  53.5,   Double.NaN,   Double.NaN},
			{130.0,  4.5,  53.5,   Double.NaN,   Double.NaN},
			{135.0,  4.5,  53.5,   Double.NaN,   Double.NaN},
			{140.0,  5.0,  53.5,   Double.NaN,   Double.NaN},
			{145.0,  5.5,  53.5,   Double.NaN,   Double.NaN},
			{150.0,  6.0,  53.5,   Double.NaN,   Double.NaN},
			{155.0,  6.5,  53.5,   Double.NaN,   Double.NaN},
			{165.0,  7.0,  53.5,   Double.NaN,   Double.NaN},
		};

		// --- Planning limits derived from the table (margin ~1 in) ---
		// Highest carriage height with the arm fully tucked (< ARM_CLEAR_MIN_ANGLE).
		public static final double ARM_TUCK_MAX_HEIGHT = 8.5;       // Inches
		// Smallest arm angle that may leave the tuck zone (5 deg is only
		// clear to ~10 in; 7.5 deg to ~21 in). L2 is 12.5 deg.
		public static final double ARM_CLEAR_MIN_ANGLE = 8.0;       // Degrees
		// Highest carriage height with the arm ANYWHERE from 8 to 75 deg
		// (the low box roof: 17.5 in at 60-70 deg is the tightest).
		public static final double LOW_BOX_ROOF = 16.5;             // Inches
		// Arm angle that clears band A, so the carriage may climb past the
		// low box roof (67.5 deg is the first continuous row).
		public static final double BAND_PASS_MIN_ANGLE = 75.0;      // Degrees
		// Highest carriage height with the arm between BAND_PASS_MIN_ANGLE
		// and SAFE_TRAVEL_MIN_ANGLE (band B for 75-95 deg starts at 36-39 in).
		public static final double MID_CORRIDOR_MAX_HEIGHT = 36.0;  // Inches
		// Arm angle from which the carriage may go anywhere: RAISE (100 deg)
		// itself; the gate accepts SAFE_ANGLE_TOLERANCE below it, i.e. a
		// measured 97 deg, and the 97.5 deg row is clear 4-53.5 in.
		public static final double SAFE_TRAVEL_MIN_ANGLE = 100.0;   // Degrees
		// Angles beyond this need height (bumper / Limelight bracket): from
		// ~113 deg up the corridor starts at 4 in, from 135 deg at 5 in and
		// at 160 deg at 7.5 in. The arm is held at this stage angle (which
		// is clear right down to the base) until the carriage is above the
		// minimum height, and brought back to it before dropping below.
		public static final double HIGH_ANGLE_STAGE = 110.0;        // Degrees
		public static final double HIGH_ANGLE_MIN_HEIGHT = 8.0;     // Inches

		// --- Mid-height scoring poses (L3): 17 < height < L4_ZONE_MIN_HEIGHT ---
		// The arm starts its final rotation (from RAISE) once the carriage is
		// this close to the target; the carriage keeps settling meanwhile.
		// 0.5 because the L3 pose (30.5 in, 25 deg) sits half an inch above
		// the 25-30 deg corridor floor (30 in).
		public static final double MID_POSE_ROTATE_BELOW_TARGET = 0.5; // Inches
		// Leaving a mid pose: lift to here (all of 25-100 deg is clear at
		// 31-35 in) while the arm swings up; only descend once the arm is at
		// BAND_PASS_MIN_ANGLE. This is the fix for the L3 return hitting the
		// middle-stage top tube when the elevator dropped too early.
		public static final double MID_POSE_RETURN_LIFT_HEIGHT = 31.0; // Inches

		// --- High scoring pose (L4): height >= L4_ZONE_MIN_HEIGHT ---
		public static final double L4_ZONE_MIN_HEIGHT = 35.0;       // Inches
		// Approach: rise at RAISE to the station, rotate to the stage angle
		// there (25-100 deg are all clear at 31-35 in), climb to the pre-top
		// height with the arm at the stage angle (25-30 deg clear to 51 in),
		// finish the rotation to 20 deg above 37 in (20 deg clear 37.5-53.5)
		// and only then send the carriage to the top.
		public static final double L4_STATION_HEIGHT = 33.0;        // Inches
		public static final double L4_ROTATE_START_HEIGHT = 31.0;   // Inches: arm may leave RAISE
		public static final double L4_STAGE_ANGLE = 25.0;           // Degrees
		public static final double L4_STAGE_DONE_ANGLE = 30.0;      // Degrees: carriage may continue up
		public static final double L4_PRE_TOP_HEIGHT = 48.0;        // Inches
		public static final double L4_FINAL_ANGLE_MIN_HEIGHT = 37.0; // Inches: arm may finish to 20 deg
		public static final double L4_FINAL_GATE_ANGLE = 22.5;      // Degrees: carriage may go to the top
		// Return ("drop before pivoting"): descend at the L4 angle to the
		// drop height (20 deg is clear 37.5-53.5), swing to 45 deg while
		// below the rotate-max height (45 deg clear 28-46), drop to the
		// station, swing to RAISE below 37 in (90 deg clear to 36.5), then
		// descend freely.
		public static final double L4_RETURN_DROP_HEIGHT = 39.0;         // Inches
		public static final double L4_RETURN_ROTATE_MAX_HEIGHT = 43.0;   // Inches
		public static final double L4_RETURN_STAGE_ANGLE = 45.0;         // Degrees
		public static final double L4_RETURN_STAGE_DONE_ANGLE = 40.0;    // Degrees
		public static final double L4_RETURN_SAFE_ROTATE_MAX_HEIGHT = 37.0; // Inches

		// --- Tolerances ---
		// The arm counts as "at" a gate angle within this tolerance (the
		// through bore reads the real arm, after the chain backlash).
		public static final double SAFE_ANGLE_TOLERANCE = 3.0; // Degrees
		// Final settle wait per planned move. Safety gates never time out.
		public static final double SETTLE_TIMEOUT_SECONDS = 3.0;
	}

	// CANdle disabled (Sept 2026): there is no CANdle on the robot, so the LED
	// subsystem (subsystems/LEDs.java) and its constants are commented out.
	// Restore both together.
	//
	// /**
	//  * Constants for the LED subsystem (CTRE CANdle).
	//  * The CANdle drives its 8 onboard LEDs plus an attached LED strip; the
	//  * onboard LEDs are indices 0-7 and the strip starts at index 8.
	//  */
	// public static final class LEDConstants {
	// 	// --- CAN IDs ---
	// 	public static final int CANDLE_ID = 5; // CANdle (rio CAN bus)
	//
	// 	// --- Strip Configuration ---
	// 	public static final int LED_COUNT = 68;       // Total LEDs: 8 onboard + strip (VERIFY - set to actual strip length)
	// 	public static final double BRIGHTNESS = 0.6;  // Global brightness scalar 0-1 (limits current draw on long strips)
	//
	// 	// --- Timing (seconds) ---
	// 	public static final double ENDGAME_WARNING_TIME = 20.0; // Teleop time remaining when the endgame pattern starts
	// }

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
			// barge: PLACEHOLDER - the CAD has its 1x1 mount (front-left,
			// ~40.6 in up, 13.4 in forward) but no camera body in it, so
			// there is nothing to derive a pose from. Measure and set
			// measured = true.
			new CameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
			// reef (front-left, base stage): FROM THE CAD (Leviathan STEP,
			// lens-barrel cylinder of the LL3G model in the "2x1
			// Perpendicular Angled Limelight Mount"): lens 11.04 in forward
			// of center, 11.25 in LEFT of center (negative in Limelight's
			// Y-right convention), 15.78 in above the floor; optical axis
			// pitched 20 deg DOWN and yawed 30 deg toward the robot's
			// centerline (to the right). VERIFY in the camera web UI 3D
			// preview (http://limelight-reef.local:5801): the model should
			// sit front-left, looking forward-right and down; if it looks
			// left instead, flip the yaw sign.
			new CameraPose(11.04 * 0.0254, -11.25 * 0.0254, 15.78 * 0.0254, 0.0, -20.0, -30.0, true),
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
