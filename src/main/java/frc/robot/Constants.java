package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// CANdle disabled (no CANdle on the robot): import com.ctre.phoenix6.signals.RGBWColor;
// CANdle disabled (no CANdle on the robot): import com.ctre.phoenix6.signals.StripTypeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.Pace;

/**
 * Every changeable number and setting on the robot, grouped by what it
 * configures. A value a person might change (a gain, a tolerance, a timeout,
 * a speed, a margin, a threshold, a rumble strength, an ID, a port) lives
 * only in this file, and every other class holds logic only, so the robot is
 * tuned in one place. What stays in other classes: NetworkTables,
 * Preferences and log keys, which stay with the code that publishes them
 * (they are identifiers the Elastic layout, AdvantageScope and the stored
 * values are bound to, so renaming one breaks its widget or loses its
 * value); true mathematical, unit and framework constants; the topic names,
 * strides and indices of an external data format (Limelight's arrays); the
 * drawing geometry of the dashboard widgets; and the vendor files
 * (generated/TunerConstants.java from CTRE Tuner X, LimelightHelpers.java,
 * util/Elastic.java).
 *
 * Conventions used throughout this file:
 *  - Every value carries its unit in the comment (inches, degrees, amps, ...)
 *  - Phoenix 6 (TalonFX) closed-loop gains are voltage-based; Spark MAX PID
 *    gains are duty-cycle-based with voltage feedforward, as noted per section
 *  - Values marked TUNE are safe starting points for on-robot testing;
 *    values marked MEASURE or VERIFY are physical measurements that must be
 *    confirmed before the gains on top of them mean anything
 *  - A value that is also a dashboard tunable (util/Tunables.java) is only
 *    the factory default: see TunablesConstants.DEFAULTS_VERSION
 *
 * =========================== TABLE OF CONTENTS ===========================
 * Nested class (in file order): what it configures / which classes read it
 *
 *   ElevatorConstants        elevator IDs, gearing, gains and profile defaults,
 *                            limits, tolerances, presets, carriage paces
 *                            read by: Elevator, Pace, Superstructure, Tunables, Dashboard
 *   CorAlConstants           pivot and roller IDs, gearing, gains and profile
 *                            defaults, coral detection, through bore, landing
 *                            correction, roller speeds, presets
 *                            read by: CorAl, Superstructure, Tunables, RobotContainer, Dashboard
 *   SuperstructureConstants  CAD clearance table, handoff heights and angles,
 *                            table walks, setpoint latch, paces per move,
 *                            score gate, eject timing, reef back-off
 *                            read by: Superstructure
 *   LEDConstants             CANdle strip, colors, animations (commented out:
 *                            no CANdle on the robot)
 *                            read by: LEDs (commented out)
 *   ControlsConstants        controller ports, driver and operator trigger
 *                            thresholds, zeroing hold, rumble cues
 *                            read by: RobotContainer
 *   DriveConstants           teleop speed and rotation rate, deadband, speed
 *                            scaling by carriage height, nudges, teleop drive
 *                            request type, SysId, swerve sim
 *                            read by: RobotContainer, Swerve, Tunables
 *   AutoConstants            PathPlanner feedback gains, path drive request
 *                            type, auto intake timeout
 *                            read by: Swerve, RobotContainer
 *   VisionConstants          cameras and their poses, field, pose-fusion gates
 *                            and std devs, heading seed, tag classes,
 *                            alignment goals, tracking gains and drive
 *                            request type
 *                            read by: Vision, Swerve, RobotContainer, Dashboard, Tunables
 *   DashboardConstants       Testing-tab intake default, low-battery alert
 *                            read by: Dashboard
 *   TunablesConstants        dashboard tunables' defaults version, poll period
 *                            and clamp ranges
 *                            read by: Tunables, Elevator, CorAl
 *   LoggingConstants         log storage, RLOG port, AdvantageScope component
 *                            poses and mechanism geometry
 *                            read by: Robot, Dashboard
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
 *
 * ============================== USB MAP =================================
 *   0       Driver Xbox controller           (ControlsConstants)
 *   1       Operator Xbox controller         (ControlsConstants)
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
        public static final boolean ELEVATOR_LEFT_INVERTED = false;      // True if positive output should be flipped (positive must move the carriage up)
        public static final boolean ELEVATOR_RIGHT_OPPOSES_LEFT = true; // True if the right motor spins opposite the left

        // --- Idle Mode (both controllers; the follower copies the leader) ---
        // Brake holds the carriage with the output off. Through 15:1 it still
        // creeps down about 3 in/s at height, so stow before disabling.
        public static final IdleMode ELEVATOR_IDLE_MODE = IdleMode.kBrake;

        // --- Current Limits (amps) ---
        public static final int ELEVATOR_CURRENT_LIMIT = 50; // Spark MAX smart current limit per NEO

        // --- Encoder Velocity Filtering (NEO hall sensor) ---
        // The Spark MAX defaults to a 32 ms measurement period averaged over 8
        // samples, which is ~130 ms of lag - fine for a dashboard readout, far
        // too slow for anything the control loop reacts to (MAXMotion restarts
        // its profile from the measured state, so a stale velocity there
        // restarts the profile at a speed the carriage no longer has). 16 ms
        // over 2 samples is ~24 ms of lag and still quiet enough to read.
        public static final int ELEVATOR_VELOCITY_PERIOD_MS = 16; // 8-64 ms
        public static final int ELEVATOR_VELOCITY_AVG_DEPTH = 2;  // 1, 2, 4 or 8
        // CAN status period for the leader's position and velocity frames.
        public static final int ELEVATOR_STATUS_PERIOD_MS = 10;

        // --- Mechanism Gearing ---
        // Power path: NEO -> 15:1 MAXPlanetary (5:1 x 3:1 cartridges) ->
        // 90-degree gearbox (1:1, REV-21-2120) -> 1/2 in hex shaft -> 22T #25
        // sprockets with chain runs at the top and bottom of the elevator.
        // (Before Sept 2026 the stack carried a second 3:1 for 45:1; gains and
        // speeds recorded for that gearing do not transfer.)
        public static final double ELEVATOR_GEAR_RATIO = 5.0 * 3.0;

        // Chain advance (inches) per sprocket rotation. #25 chain has a
        // 0.25 in pitch, so a 22T sprocket advances the chain exactly
        // 22 x 0.25 = 5.5 in per rotation.
        public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE = 22 * 0.25;

        // Cascade rigging: the chain lifts the middle stage, and the cascade
        // run doubles that motion at the carriage - the carriage travels 2x
        // the chain advance.
        public static final double ELEVATOR_CASCADE_RATIO = 2.0;

        // Carriage travel (inches) per motor rotation predicted by the
        // gearing alone: 5.5 x 2 / 15 = ~0.733.
        public static final double ELEVATOR_MODELED_INCHES_PER_ROTATION =
            ELEVATOR_SPROCKET_CIRCUMFERENCE * ELEVATOR_CASCADE_RATIO / ELEVATOR_GEAR_RATIO;

        // measured correction on top of the model - the factory default of
        // the "Elevator - Travel Ratio" tunable (the live value is what the
        // Spark MAX gets; see Elevator.inchesPerRotation()).
        //
        // CONFIRMED 1.0 on the robot (Sept 2026). Five tape tests, measured
        // from the top of the base-stage 2x1 to the bottom of the carriage
        // 2x1, all fit one line: measured = model travel + a constant, and
        // that constant is the 1 in middle-stage tube (the tape readings put
        // it near 0.875 - they were 1/8 in optimistic).
        //   commanded   ratio applied   model travel   tape
        //      2.00        1.0              2.00        ~2.75
        //      6.00        1.375            4.36         5.125
        //     20.00        1.17            17.03        ~17.9
        //     40.00        1.17            34.05        ~35
        // (the fifth, an 8 in test that read 6.375, was taken while the
        // elevator was not moving cleanly and is the one outlier). Between
        // the 20 and 40 in tests the slope is 1.005, so the gearing model is
        // right to within 0.5%; the constant is not a scale error but the
        // height the carriage sits at on its hard stop in that measurement
        // frame - see ELEVATOR_ZERO_HEIGHT. The trial ratios in the table
        // (1.375, 1.17) were single-point fits to that offset: never set this
        // ratio from one tape reading - take two heights and use the slope.
        public static final double ELEVATOR_MEASURED_TRAVEL_RATIO = 1.0;

        // --- MAXMotion Profile (inches, seconds) - tunable defaults ---
        // NEO free speed 5676 RPM = 94.6 rot/s -> 94.6 x 0.733 = ~69 in/s
        // theoretical top speed through 15:1. Cruise is 58% of that: kV x 40 =
        // ~6.9 V, plus kG, kS and kA x 200 = ~0.8 V during the acceleration
        // ramp, so a full-speed climb peaks near 9 V and keeps ~3 V of headroom
        // for the position loop on a sagging battery. Full 52 in of travel
        // takes ~1.5 s. (The Tunables getters clamp the live values to 60 in/s
        // and 500 in/s^2: TunablesConstants.)
        // Acceleration is the mast's comfort limit, not the motors': the
        // trapezoidal profile changes commanded acceleration in a step at the
        // end of every move (MAXMotion has no S-curve in REVLib 2026, so there
        // is no jerk limit to soften it) and the structure rings at whatever
        // that step is. At 300 in/s^2 the elevator visibly shakes as it stops;
        // 200 in/s^2 cuts the stopping force from ~173 N to ~116 N at the
        // carriage and costs only ~0.07 s on a full-travel move. It is also
        // the deceleration the staged L3/L4 sequences see when the carriage
        // stops at a station, so do not raise it casually.
        // The profile always decelerates into the setpoint, so the carriage
        // settles rather than hitting the target. Adjust on the Testing tab
        // ("Elevator - Cruise Velocity" / "Max Acceleration"). The
        // Superstructure's staged sequences are gated on measured state, so
        // they stay safe at any speed - only their duration changes.
        public static final double ELEVATOR_MAX_VELOCITY = 40.0;      // Cruise velocity (in/s)
        public static final double ELEVATOR_MAX_ACCELERATION = 200.0; // Acceleration (in/s^2)
        // How far the carriage may stray from the MAXMotion profile before the
        // controller regenerates the profile from the current position and the
        // measured velocity. Not a settling tolerance. With kA carrying the
        // acceleration the carriage tracks the profile within ~0.1 in, so this
        // window should fire only on a real disturbance (a stall or a
        // collision) - a window tight enough to fire on an ordinary
        // acceleration ramp turns one move into a series of regenerated
        // profiles. 1.0 rather than 0.5: under a sagging bus the output
        // saturates near cruise, the error passes half an inch, and the
        // regenerated profile (built on a lagged velocity) then hunts. Every
        // safety gate reads the measured height, so a wider window costs
        // nothing.
        public static final double ELEVATOR_ALLOWED_PROFILE_ERROR = 1.0;

        // --- Closed-Loop Gains (Spark MAX slot 0; error in inches) - tunable defaults ---
        // kP, kS, the kV scale, kA, kG, and the profile above are all editable
        // on the Testing tab and re-applied to both controllers the next time
        // the robot is disabled - no redeploy, no REV Hardware Client.
        // TUNE - procedure after any mechanical change:
        //   1. Calibrate the height reading against a tape measure (README:
        //      "Calibrating the elevator height").
        //   2. Set kG from "Elevator/kG From Cruise": with the arm at RAISE,
        //      run one long move each way from the Testing tab; the readout
        //      is the mean of the applied volts at cruise going up and going
        //      down, which friction cancels out of. ("Elevator/Hold Volts" is
        //      a cross-check only: inside the static-friction band the hold
        //      voltage partly echoes the gains already configured.)
        //   3. Command a preset and check the carriage tracks without lagging
        //      on the ramps (that is kA) or resting short (that is kP or kG).
        // kP sizing: the position loop must supply whatever the feedforward
        // does not, and at 15:1 one inch of error is only 1.4 motor rotations,
        // so a duty-per-inch figure is soft at the carriage: 0.4 duty/in is
        // ~700 N per inch of error. That is firm enough that friction and a
        // mis-set kG leave about 0.1 in of rest error or less. With kD 0 the
        // loop's only damping is the NEO's back-EMF: the damping ratio is
        // ~0.63 at 0.4 (CAD mass estimate) against ~0.56 at 0.5, where every
        // disturbance at a setpoint rings for two or three visible cycles at
        // ~6 Hz; the price is ~0.02 in more rest error. kD is not an option:
        // the hall encoder resolves 0.0175 in, and a derivative of that at
        // 1 kHz is noise. If the carriage wobbles at a setpoint, 0.3 is ~0.72.
        // Symptoms: rests short of target -> raise kP, or fix kG (step 2);
        // buzzes or hunts at rest -> lower kP; overshoots on the way into a
        // target -> lower kA; lags behind on the ramps -> raise kA.
        public static final double ELEVATOR_kP = 0.4; // Duty cycle per inch of position error
        public static final double ELEVATOR_kI = 0.0; // Leave 0 - kG handles gravity sag
        public static final double ELEVATOR_kD = 0.0; // Duty cycle per in/s of error derivative

        // --- On-Controller Feedforward (volts; REVLib FeedForwardConfig) ---
        // kS: zero, on purpose. (REV's definition is the largest voltage that
        // does not move the carriage.) The Spark MAX applies kS as +kS
        // whenever the profile velocity is zero (measured in REV's own
        // simulator: it does not follow the sign of the error), so a non-zero
        // kS (a) biases the hold voltage to kG + kS, and (b) steps the
        // feedforward by +2 x kS at the instant a descent ends (kG - kS while
        // moving down, kG + kS at rest). That step, into a lightly damped
        // loop, shows up as a wobble around every setpoint that is reached
        // from above. Moving friction is ~0.2 V, which the position loop
        // covers with ~0.04 in of lag, so nothing is lost by leaving it out.
        public static final double ELEVATOR_kS = 0.0;
        // kA: volts per in/s^2 of profile acceleration (REVLib applies it in
        // MAXMotion modes only). Without it the position loop has to supply
        // the whole acceleration force out of tracking error: at 15:1 the
        // carriage plus arm plus the reflected rotor inertia is ~23 kg
        // effective, so the 200 in/s^2 ramps need ~0.8 V. Bought with error
        // instead, that is lag on the way up and lead on the way down - the
        // carriage runs past its target and, if the error passes the allowed
        // profile error, the profile regenerates around it.
        // 0.0040 = 12 V / 105 A x (0.733 in/rot / 2pi x 0.0254) /
        // (2 x 0.0248 Nm/A) x 23 kg x 0.0254. Raise it if the carriage lags
        // on the ramps, lower it if it leads.
        public static final double ELEVATOR_kA = 0.0040;
        // kV is not a stored constant. It is the NEO back-EMF model,
        // 12 V / (free speed in in/s), and free speed in inches depends on
        // the travel ratio - so the Elevator derives it from the live ratio
        // (modelKv: ~0.17 V per in/s at 0.733 in/rot through 15:1). The kV
        // scale multiplies that model (REV: overshoot -> reduce kV).
        public static final double NEO_FREE_SPEED_RPM = 5676.0;
        public static final double ELEVATOR_kV_SCALE = 1.0;
        // kG: nothing counterbalances this elevator (the constant-force
        // springs in the CAD are not on the robot), so the motors hold the
        // whole carriage + arm + half the middle stage - about 145 N by the
        // CAD mass estimate, which is ~1.0 V through 15:1 (the estimate spans
        // 0.8-1.3 V). 1.0 is that estimate, NOT a measurement: set it from
        // the "Elevator/kG From Cruise" readout (TUNE step 2). If a
        // counterbalance is ever added, re-measure - kG drops by whatever
        // share it carries.
        public static final double ELEVATOR_kG = 1.0;
        // "Elevator/kG From Cruise" samples the applied volts only while the
        // measured speed is within this fraction of the cruise velocity
        // (steady cruise, off the ramps), and low-pass filters each direction
        // with this factor per 20 ms loop. TUNE: a wider window lets the
        // ramps' kA x acceleration into the reading.
        public static final double ELEVATOR_CRUISE_SAMPLE_WINDOW = 0.05; // Fraction of cruise velocity
        public static final double ELEVATOR_CRUISE_VOLTS_FILTER = 0.1;   // Filter factor per loop, 0-1

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
        // the carriage does not sit at 0 in that frame: the middle-stage tube
        // is in the gap, and it is a 2x1 lying on its 1 in side, so the gap is
        // exactly the tube: 1.000 in (team, Sept 2026). The CAD agrees - the
        // base stage's bottom cross tube tops out at Y 5.875, the middle
        // stage's 1 in tube sits on it, and the carriage tube bottom is at
        // 6.875 - and so does the CAD clearance analysis, which is computed
        // with the carriage resting at 1.000. (Five tape tests read ~0.875;
        // they were an eighth of an inch optimistic.) The encoder is
        // referenced TO this value, not zeroed, every time the carriage is at
        // its hard stop, so a commanded 20 in lands the tape at 20 in.
        // Default of the "Elevator - Height At Hard Stop" tunable. The
        // reverse soft limit is this height; presets of 0 clamp to it.
        public static final double ELEVATOR_ZERO_HEIGHT = 1.0;
        public static final double ELEVATOR_MAX_POSITION = 53.0; // Forward soft limit

        // --- Tolerances (inches) ---
        // "At target": what the Superstructure waits for before it calls a
        // pose reached, and what the dashboard's At Target light shows. The
        // hall encoder resolves 0.0175 in, and the loop's rest band is
        // (friction volts + kG error) / (12 x kP), about 0.03-0.11 in. The
        // window has to sit outside that band - a tighter one (0.05 in) lies
        // inside it and the light flickers. This is a "pose reached" flag
        // only - no safety gate reads it. If a pose ever sits waiting, the
        // loop is resting outside this window: fix kG (Elevator/kG From
        // Cruise) or raise kP rather than loosening this. Judge the rest
        // error from Elevator/Height against Elevator/Target.
        public static final double ELEVATOR_ALLOWED_ERROR = 0.25;
        // Left-vs-right travel disagreement that raises the out-of-sync alert.
        // Both sides drive the same hex shaft, so this is a slip detector, not
        // a precision figure: one skipped #25 tooth is 0.25 in of chain = 0.5
        // in of carriage, and each encoder quantises at 0.0175 in.
        public static final double ELEVATOR_SIDE_SYNC_TOLERANCE = 0.25;
        // The encoder must read within this of the hard-stop height for a
        // calibration edit (travel ratio / hard-stop height) to be applied:
        // the carriage is then on its hard stop and is re-referenced there.
        public static final double ELEVATOR_AT_BASE_TOLERANCE = 0.5;
        // A reading this far below the hard-stop height means the encoder was
        // referenced with the carriage raised and it has since dropped to
        // the hard stop - every commanded height would land that much high.
        // The Dashboard alerts to re-zero.
        public static final double ELEVATOR_BELOW_ZERO_ALERT = -0.25;

        // --- Manual Control (unitless stick values) ---
        public static final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
        // "Hold here" keeps the setpoint the carriage already has when it is at
        // rest (slower than ELEVATOR_HOLD_KEEP_MAX_SPEED) within this of it,
        // instead of issuing a new one at the measured height (a zero-length
        // profile). TUNE.
        public static final double ELEVATOR_HOLD_KEEP_WINDOW = 0.5;    // Inches
        public static final double ELEVATOR_HOLD_KEEP_MAX_SPEED = 1.0; // in/s
        public static final double ELEVATOR_MANUAL_MAX_VOLTS = 3.0;        // Full stick = kG +/- this (about 17 in/s either way)

        // --- Paces (fraction of the tuned cruise velocity and acceleration) ---
        // One Spark MAX closed-loop slot per Pace (subsystems/Pace.java); a
        // move at scale s takes 1 / s as long. The Superstructure picks the
        // pace per move (SuperstructureConstants.*_PACE). FULL is the tuned
        // profile itself (1.0 by definition, in Pace.java, not a setting).
        // TUNE the others with SuperstructureSequenceSimTest.pacesCompared
        // (build/sim/paces.csv), and keep each below 1.0: the scale multiplies
        // the live profile after the Tunables clamp, so a pace above FULL
        // could run the carriage past that clamp.
        public static final double PACE_BRISK_SCALE = 0.8;
        public static final double PACE_EASY_SCALE = 0.6;
        public static final double PACE_SLOW_SCALE = 0.4;

        // --- Desktop Simulation (affects the sim GUI only, never the robot) ---
        public static final double ELEVATOR_SIM_CARRIAGE_MASS_KG = 6.0; // kg; estimate (MEASURE only if sim fidelity matters)

        // --- Preset Heights (inches; bottom of carriage 2x1 to top of base stage 2x1) ---
        public enum PresetHeights {
            BASE(0.0),
            CORAL_L1(0.0),
            CORAL_L2(12.0),
            CORAL_L3(30.5),
            // 51.5 keeps 1.5 in to the forward soft limit (53.0) and 1 in under
            // the 20 deg corridor ceiling (52.5). Holding at 52.5 leaves half
            // an inch to the soft limit and sits exactly on that ceiling, and
            // the carriage stutters against the top of travel. Raise it toward
            // 52 if the coral needs the height, but not past it.
            CORAL_L4(51.5),
            ALGAE_LOW_INTAKE(20.5),
            ALGAE_HIGH_INTAKE(37.5),
            // 52.0 keeps a full inch under the 53.0 soft limit; at half an inch
            // (52.5) the carriage stutters against the top of travel. The barge
            // shot wants height - raise it toward 52.5 only if the shot needs
            // it and the carriage holds there quietly.
            ALGAE_SCORE(52.0);

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
        // Phoenix 6 device IDs only go up to 62: constructing a device with a
        // larger ID throws and crashes robot code on boot (RobotContainerTest
        // constructs every device, so it catches this).
        public static final int CANRANGE_SENSOR_ID = 3;    // CANrange game piece sensor

        // --- DIO Ports ---
        public static final int THROUGH_BORE_DIO_PORT = 0; // Through bore encoder input

        // --- Motor Inversion ---
        public static final boolean CORAL_PIVOT_MOTOR_INVERTED = false; // True if positive output should be flipped (positive must raise the arm)
        public static final boolean CORAL_INTAKE_MOTOR_INVERTED = true; // True if positive output should be flipped (positive must intake coral)

        // --- Neutral Modes ---
        public static final NeutralModeValue CORAL_PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake;  // Holds the arm with the output off
        public static final NeutralModeValue CORAL_INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake; // Rollers stop at once and grip a held piece

        // --- Current Limits (amps) ---
        public static final int CORAL_PIVOT_CURRENT_LIMIT = 30;  // Pivot supply current limit (breaker protection)
        // Stator limit caps the pivot's torque (a Kraken through 65:1 can
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
        // Defaults for the live tunables "Pivot - Cruise Velocity" /
        // "Acceleration" / "Jerk" (Testing tab), re-applied to the TalonFX
        // the next time the robot is disabled.
        //
        // Chain backlash (Sept 2026): the pivot chain has slack, so at the end
        // of a fast move the arm coasts through the slack and the chain
        // catches it, which jolts the whole assembly. Motion Magic decelerates
        // at the same rate it accelerates, so the acceleration sets how hard
        // that catch is, and the jerk limit sets how abruptly the
        // deceleration begins.
        // The profile is sized from what the sweeps need. The rotations the L3
        // and L4 sequences wait on are about 75 deg, and a short move is
        // limited by acceleration, not top speed (75 deg at 300 deg/s^2 peaks
        // at 150 deg/s and never reaches a 200 deg/s cruise), so acceleration
        // is what shortens them. Sweep times, cruise (deg/s) / accel (deg/s^2):
        //     sweep      200/300      300/800 (the defaults)
        //     75 deg     1.30 s       0.88 s
        //     100 deg    1.45 s       0.97 s
        //     160 deg    1.77 s       1.18 s
        //   - 300 deg/s is 55% of the Kraken's free speed through 65.41:1
        //     (100 rotor rps / 65.41 = 1.53 rot/s = 550 deg/s), so kV at cruise
        //     is ~6.5 V and kP still has room.
        //   - 800 deg/s^2 costs the motor about 9 A against the arm's inertia -
        //     nothing. What it costs the game piece is the real limit: peak
        //     centripetal load at the claw is ~1.0 g (~0.4 g at 200/300).
        //   - 6000 deg/s^3 gives a 0.13 s S-curve ramp (acceleration / jerk),
        //     so every stop ramps in instead of snapping - that is what the
        //     chain slack needs.
        // If a held piece slips: on a long swing (algae, base to RAISE) the
        // peak is the cruise, so lower cruise; on the short scoring rotations
        // the peak is set by acceleration, so lower acceleration.
        // The Superstructure handoff heights follow the live values - re-check
        // the Handoffs readout after a change.
        public static final double CORAL_PIVOT_MAX_VELOCITY = 300.0;     // Cruise velocity (deg/s)
        public static final double CORAL_PIVOT_MAX_ACCELERATION = 800.0; // Acceleration (deg/s^2)
        public static final double CORAL_PIVOT_MAX_JERK = 6000.0;        // Jerk limit (deg/s^3); 0.13 s to reach full accel
        // ArmAxis.setProfileScale runs the whole profile slower (cruise x s,
        // acceleration x s^2, jerk x s^3) so a sweep can be paced to the
        // carriage. Never below this fraction of the tuned profile (TUNE), and
        // a change smaller than the deadband is not sent, since each send is
        // a CAN config write.
        public static final double PIVOT_PROFILE_SCALE_MIN = 0.3;       // Fraction of the tuned profile
        public static final double PIVOT_PROFILE_SCALE_DEADBAND = 0.01; // Fraction of the tuned profile

        // --- Closed-Loop Gains (Phoenix 6 slot 0; voltage-based, error in mechanism rotations) ---
        // TUNE - procedure after any mechanical change:
        //   1. Sanity-check the angle reading against the through bore /
        //      a protractor (ratio is derived from real tooth counts).
        //   2. Command a preset; raise kP until tracking is crisp, add kD if
        //      it oscillates.
        //   3. Measure the balance angle and kG (procedure at kG below).
        // kP sanity: 40 V/rot is 0.11 V per degree - the 1-degree at-target
        // tolerance is worth only 0.11 V, so this cannot ring on its own; the
        // profile feedforwards below do the moving and kP only corrects error.
        // Symptoms: stops short of the angle -> raise kP; buzz at rest -> lower.
        public static final double CORAL_PIVOT_kP = 40.0; // Volts per rotation of position error (~0.11 V/deg)
        public static final double CORAL_PIVOT_kI = 0.0;  // Leave 0 - use kG for gravity
        public static final double CORAL_PIVOT_kD = 0.0;  // Volts per rot/s of error derivative
        // kG does not need 0 deg to be horizontal: Phoenix 6 (26.x) has
        // Slot0.GravityArmPositionOffset, and Arm_Cosine then outputs
        // kG x cos(position + offset). This arm's 0 deg has the claw ~33 deg
        // past vertical toward the rear, so gravity does nothing at the
        // balance angle (~33 deg, claw straight up) and the offset is
        // 0.25 rot - balance/360 (checked for sign in the Phoenix sim: +57 deg
        // of offset gives +0.55 V at 0 deg, 0 at 33, -1.0 at 123 for kG = 1).
        // kG STAYS 0 UNTIL MEASURED - with 0 the offset does nothing:
        //   1. Balance angle: disabled, pivot in Coast (Tuner X), find the
        //      through-bore angle where the claw balances.
        //   2. kG: at balance + 90 (~123 deg, claw horizontal) creep up and
        //      down at ~5 deg/s and read the pivot's MotorVoltage:
        //      kG = (|V up| + |V down|) / 2, kS = the half-difference.
        // Expect ~0.3 V (4 kg at ~0.25 m through 65.41:1). Without it the arm
        // rests 2-3 deg low at RAISE and the algae poses (kG / kP).
        public static final double CORAL_PIVOT_kG = 0.0;  // Volts to hold the arm with the claw horizontal
        public static final double CORAL_PIVOT_BALANCE_ANGLE_DEG = 33.0; // Degrees: where gravity does nothing (0 < angle < 180); an estimate until measured (step 1)

        // --- Profile Feedforward (volts) ---
        // Physics-based so Motion Magic tracks its profile instead of lagging
        // it and catching up (which is what shows up as overshoot/oscillation
        // at the end of a move). kV = 12 V / free speed at the mechanism
        // (100 rotor rps / 65.41 = 1.53 pivot rot/s). kS overcomes static
        // friction so the arm starts moving without kP winding up first.
        public static final double CORAL_PIVOT_kS = 0.2;  // Volts to overcome static friction (TUNE: raise until motion starts)
        public static final double CORAL_PIVOT_kV = 7.85; // Volts per pivot rot/s of profile velocity
        // kA: volts per pivot rot/s^2 of profile acceleration. Motion Magic
        // applies it on the ramps, so the position loop does not have to buy
        // that force with tracking error (lag on the way in, overshoot at the
        // end - see ELEVATOR_kA). 800 deg/s^2 is 2.2 rot/s^2 and needs ~0.3 V
        // against the arm's inertia (~0.8 kg m^2 through 65.41:1), hence 0.13.
        public static final double CORAL_PIVOT_kA = 0.13;

        // --- Pivot Angle Limits (degrees) ---
        // The pivot is zeroed at its base position (CorAl stores that zero
        // against the through bore's absolute reading and restores it at
        // every start), so the base IS zero; there is no travel below it.
        public static final double CORAL_PIVOT_MIN_ANGLE = 0.0;   // Reverse soft limit
        public static final double CORAL_PIVOT_MAX_ANGLE = 160.0; // Forward soft limit

        // --- Tolerances (degrees) ---
        // "At target" is measured on the through bore (the real arm angle),
        // while the closed loop runs on the motor sensor, so this window has
        // to cover the chain's backlash and stretch as well as the loop's own
        // error. 1.0 deg is 0.27 in at the claw.
        // The Superstructure's settle also releases once both mechanisms have
        // stopped, so a tight window here costs precision-chasing time only
        // when the arm is genuinely still moving.
        public static final double CORAL_PIVOT_ALLOWED_ERROR = 1.0;

        // --- Manual Control (unitless stick values) ---
        public static final double CORAL_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
        public static final double CORAL_MANUAL_SPEED_LIMIT = 0.1;      // Max duty cycle in manual mode

        // --- Game Piece Detection (CANrange) ---
        // The CANrange's own proximity bit is "distance < threshold", with a
        // hysteresis band around it, and only when the return is strong
        // enough to count as a valid measurement. The empty claw's own
        // structure returns a reading close to a held coral's, so a verdict
        // that sits near that background chatters, and a rising-only debounce
        // latches the chatter as a coral. The verdict is therefore formed in
        // code (CorAl.readDetection): a signal-strength gate, a measurement
        // health check, a hysteresis band around the distance threshold, a
        // narrow field of view so oblique structure is not in the beam, and a
        // debounce on both edges. The threshold, hysteresis and strength gate
        // are live tunables ("CorAl - Coral Detect ..." and "CorAl - Coral Min
        // Signal Strength"); set them from the CorAl/CANrange Distance and
        // CorAl/CANrange Signal Strength readouts, claw empty and coral held.
        public static final double GAME_PIECE_DETECTION_CONFIRMATION_TIME = 0.3; // Seconds each edge must persist (both edges)
        // MEASURED on the robot (Sept 2026), and it settles how this claw
        // detects a coral. CTRE's CANrange tuning guide describes two setups;
        // this one is the first, "open air":
        //                      distance        signal strength
        //   coral held         0.04-0.05 m     65535 (saturated)
        //   claw empty         about the same  3000-4000
        // The distance is therefore useless here - an empty claw returns a
        // short reading of nothing in particular - and the signal strength is
        // what separates the two, by a factor of sixteen. So the strength gate
        // does the work and the distance threshold is deliberately generous:
        // per the guide, it only has to sit above the distance a held piece
        // reads. CTRE's default strength gate (2500) is no use here: the empty
        // claw's 3000-4000 clears it, and an empty claw reports a coral.
        public static final double GAME_PIECE_MIN_SIGNAL_STRENGTH = 15000;       // Tunable default: ~4x the empty return, far below a held one
        public static final double GAME_PIECE_DETECTION_THRESHOLD = 0.30;        // Tunable default: well above the 0.05 m a held coral reads (m)
        public static final double GAME_PIECE_DETECTION_HYSTERESIS = 0.015;      // Tunable default: the verdict only changes outside threshold +/- this (m)
        // A return that has already counted as a coral keeps counting until the
        // strength falls this far below the gate, so a piece held at an awkward
        // angle cannot blink in and out.
        public static final double GAME_PIECE_STRENGTH_RELEASE_FRACTION = 0.75;
        // Which side of the distance threshold a held coral puts the reading on.
        // With the strength gate doing the work this barely matters, but the
        // CANrange's own proximity bit can only mean "closer than the
        // threshold", so the verdict is formed in code and this picks the
        // polarity for a claw where that is the wrong question.
        public static final boolean GAME_PIECE_DETECT_WHEN_CLOSER = true;
        // Master switch. Turn detection off (tunable 0) to run the intake purely
        // on the operator's button while the thresholds are still being set: a
        // sensor stuck on "detected" otherwise makes the intake command skip
        // itself, which looks like a dead button.
        public static final boolean GAME_PIECE_DETECTION_ENABLED = true;
        public static final double GAME_PIECE_FOV_DEGREES = 13.5;                // Field of view, both axes (6.75 min, 27 max); narrow keeps claw structure out of the beam
        // Short-range mode at 100 Hz: the coral sits centimeters away, and
        // short range is the more reliable mode at that distance.
        public static final UpdateModeValue GAME_PIECE_UPDATE_MODE = UpdateModeValue.ShortRange100Hz;

        // --- Roller Speeds (duty cycle, -1 to 1; positive = coral intake direction) ---
        public static final double CORAL_INTAKE_SPEED = 0.1;   // Intaking coral (auto-stops on detection)
        public static final double CORAL_SCORE_SPEED = 0.3;    // Ejecting coral at L2-L4: on through the claw, the intake's direction
        // L1 is the odd one out: the coral goes back out the way it came in,
        // so the rollers run against the intake direction (negative). The
        // direction is confirmed on the robot; -0.3 is a starting value (TUNE).
        public static final double CORAL_L1_SCORE_SPEED = -0.3;
        public static final double ALGAE_INTAKE_SPEED = -0.1;  // Intaking algae (reverse direction)
        public static final double ALGAE_HOLD_SPEED = -0.025;  // Holding pressure on algae
        public static final double ALGAE_SCORE_SPEED = 0.5;    // Ejecting algae

        // --- Through Bore Encoder (degrees) ---
        // The REV Through Bore reports one full revolution (360 deg)
        // across its duty cycle range. This assumes the encoder is mounted
        // 1:1 on the pivot shaft; if it is ever moved to a geared/chained
        // shaft, scale this by that stage's ratio.
        public static final double THROUGH_BORE_DEGREES_PER_ROTATION = 360.0;
        public static final double THROUGH_BORE_ALLOWED_DISCREPANCY = 2.0; // Max motor-vs-through-bore disagreement before re-sync
        // The through-bore zero is stored on the roboRIO and restored at every
        // start, so a restart with the arm raised keeps the real angle. A
        // restored angle outside this range is refused (the encoder has moved
        // on its shaft) and the dashboard raises an alert; the range reaches
        // a little past each end of travel. VERIFY after any work on the
        // through bore mount.
        public static final double PIVOT_BOOT_RESTORE_MIN_ANGLE = -10.0; // Degrees
        public static final double PIVOT_BOOT_RESTORE_MAX_ANGLE = CORAL_PIVOT_MAX_ANGLE + 10.0; // Degrees
        // A restored angle farther than this from the base raises the
        // dashboard's "started away from its base" info alert. TUNE.
        public static final double PIVOT_BOOT_RESTORED_ALERT_DEG = 3.0; // Degrees
        // The rotor is re-seeded from the through bore at the start of a move
        // only if the arm is slower than this: a seed taken at speed is stale
        // by the sensor + CAN latency and steps the closed loop's feedback.
        public static final double PIVOT_RESEED_MAX_VELOCITY = 10.0; // deg/s
        // Landing correction. The loop closes on the rotor, the chain has
        // slack, and at the end of a sweep the arm's inertia (and gravity)
        // carry it through that slack: the rotor is on target and the real
        // arm rests a few degrees past it, for good. Once the arm has been
        // still for a moment, if the through bore is off the target by more
        // than the tolerance, the rotor is re-seeded from the through bore and
        // the same target re-issued, so the loop drives out the difference -
        // at most this many times per target, so it cannot hunt inside the
        // slack. An error beyond the maximum is not backlash; it is left for
        // the feedback alert.
        public static final double PIVOT_LANDING_STILL_DEG_S = 3.0;
        public static final double PIVOT_LANDING_STILL_SECONDS = 0.2;
        public static final double PIVOT_LANDING_TOLERANCE_DEG = 0.75;
        public static final double PIVOT_LANDING_MAX_DEG = 12.0;
        public static final int PIVOT_LANDING_MAX_CORRECTIONS = 3;
        public static final double PIVOT_LANDING_STEADY_DEG = 0.5;   // Through bore must stay inside this band to count as still
        public static final double PIVOT_LANDING_END_ZONE_DEG = 3.0; // No corrections this close to either end of travel

        // --- Desktop Simulation (affects the sim GUI only, never the robot) ---
        public static final double CORAL_SIM_ARM_LENGTH_METERS = 0.4; // Meters; estimate (MEASURE only if sim fidelity matters)
        public static final double CORAL_SIM_ARM_MASS_KG = 4.0;       // kg; estimate (MEASURE only if sim fidelity matters)

        // --- Preset Angles (degrees) ---
        // 0 deg is the CAD's intake pose: the claw points up and ~33 deg past
        // vertical toward the rear (the funnel); positive angles rotate it
        // forward. RAISE is the safe travel angle: per the CAD clearance map
        // (SuperstructureConstants) 100 deg is clear from ~4 in to the top,
        // while 90 deg is blocked above ~36.5 in by the middle-stage top
        // tube meeting the claw's lower rear.
        public enum PivotPresetAngles {
            BASE(0.0),          // Coral intake position
            RAISE(100.0),       // Safe travel / algae hold position
            CORAL_L1(100.0),
            // L2 scores at 12.5 rather than the geometric 5: the pivot chain
            // has backlash, so the arm can sag a few degrees past where the
            // rotor thinks it is. At 12 in the CAD gives 10 deg a full inch
            // to the cross bar but only 0.5 in at 7.5; 12.5 keeps the inch
            // even 2.5 deg low. Revisit if the chain slack is taken out.
            CORAL_L2(12.5),
            // L3 is 25 deg at 30.5 in: the nearest pose to the geometric one
            // (22.5 deg at 29 in, which the CAD puts ~0.25 in from the
            // middle-stage top sprocket shaft) that has a full inch - claw
            // tip 0.7 in further forward and 1.6 in higher. 27.5 deg at
            // 30.5 in has 1.75 in if it ever rubs.
            CORAL_L3(25.0),
            // L4 cannot be steeper than this. From ~25 to ~85 deg the claw's
            // lower rear meets the middle-stage top tube and spring hardware
            // at the top of travel, so a 35 or 45 deg L4 is not reachable.
            // The top of travel is clear only at <= 22.5 deg; 20 gives
            // ~1.25 in of model clearance (15-20 all work if the coral
            // wants a different tilt - stay at or below 22.5).
            CORAL_L4(20.0),
            // Intentionally equal to the forward soft limit (the
            // mechanical max). The soft limit only cuts output past the
            // threshold and the arm approaches from below with gravity
            // pulling back, so settling exactly at it works; if the arm
            // hunts against the limit during tuning, drop this 1-2 deg.
            ALGAE_INTAKE(160.0),
            ALGAE_SCORE(105.0);

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
     * middle stage (which rises at half the carriage travel), the funnel,
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
     *   - Above band A there is a high corridor for 20-65 deg whose ceiling
     *     is where the claw's lower rear (motor bracket) meets the
     *     middle-stage top tube and constant-force-spring hardware again
     *     ("band B": ~41 in at 60 deg, ~46 in at 45 deg, ~51 in at 30 deg).
     *     At the top of travel only angles <= 22.5 deg (or >= ~97 deg) are
     *     clear. That is why the L4 pose is 20 deg and why the arm can never
     *     rotate down to L4 near the top - the rotation happens at the
     *     33 in "station" instead.
     *   - 90 deg is NOT clear at every height: band B blocks it from
     *     ~36.5 in up. 100 deg is clear from ~4 in to the top, so RAISE (the
     *     safe travel / algae hold angle) is 100.
     *   - Angles > 130 deg hit the bumper / brainpan below ~5-7.5 in.
     *
     * FREE_CORRIDORS is that map with a 1.0 in model margin (about 1.0-1.5 in
     * real). The planner's staged sequences were simulated against the full
     * CAD model at an elevator profile of 50 in/s and 400 in/s^2 (faster than
     * the 40 / 200 defaults), sampled every 10 ms: every transit clears by
     * >= 1.0 in, also with the arm slowed to 150 deg/s. The sequences are
     * gated on measured height and angle, so clearance does not depend on the
     * profile speeds, and SuperstructureSequenceSimTest re-checks every
     * ordered pair of operator poses against this table. Every preset is
     * inside a corridor (audited at startup: Superstructure.presetAuditMessage).
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
        // Highest carriage height with the arm anywhere from 8 to 75 deg
        // (the low box roof: 17.5 in at 60-70 deg is the tightest).
        public static final double LOW_BOX_ROOF = 16.5;             // Inches
        // Arm angle that clears band A, so the carriage may climb past the
        // low box roof (67.5 deg is the first continuous row).
        public static final double BAND_PASS_MIN_ANGLE = 75.0;      // Degrees
        // Highest carriage height with the arm between BAND_PASS_MIN_ANGLE
        // and SAFE_TRAVEL_MIN_ANGLE (band B for 75-95 deg starts at 36-39 in).
        public static final double MID_CORRIDOR_MAX_HEIGHT = 36.0;  // Inches
        // Arm angle from which the carriage may go anywhere: RAISE (100 deg)
        // itself. The gate accepts SAFE_ANGLE_TOLERANCE below it (a measured
        // 97 deg): with chain slack the arm rests a degree or two short of
        // RAISE, and a table lookup from there is nudged onto the RAISE row
        // (Superstructure.snapIntoTable).
        public static final double SAFE_TRAVEL_MIN_ANGLE = 100.0;   // Degrees
        // Angles beyond this need height (bumper / Limelight bracket): from
        // ~113 deg up the corridor starts at 4 in, from 135 deg at 5 in and
        // at 160 deg at 7.5 in. The arm is held at this stage angle (which
        // is clear right down to the base) until the carriage is above the
        // minimum height, and brought back to it before dropping below.
        public static final double HIGH_ANGLE_STAGE = 110.0;        // Degrees
        public static final double HIGH_ANGLE_MIN_HEIGHT = 8.0;     // Inches

        // --- Mid-height scoring poses (L3): LOW_BOX_ROOF < height < L4_ZONE_MIN_HEIGHT ---
        // The arm starts its final rotation (from RAISE) once the carriage is
        // this close to the target; the carriage keeps settling meanwhile.
        // 0.5 because the L3 pose (30.5 in, 25 deg) sits half an inch above
        // the 25-30 deg corridor floor (30 in).
        public static final double MID_POSE_ROTATE_BELOW_TARGET = 0.5; // Inches
        // Leaving a mid pose: lift to here (all of 25-100 deg is clear at
        // 31-35 in) while the arm swings up; only descend once the arm is at
        // BAND_PASS_MIN_ANGLE. A carriage that drops before then puts the claw
        // into the middle-stage top tube.
        public static final double MID_POSE_RETURN_LIFT_HEIGHT = 31.0; // Inches

        // --- High scoring pose (L4): height >= L4_ZONE_MIN_HEIGHT ---
        public static final double L4_ZONE_MIN_HEIGHT = 35.0;       // Inches
        // Approach: the arm leaves RAISE as the carriage passes
        // ARM_RELEASE_MIN_HEIGHT and sweeps down under the arm-side clamp
        // while the carriage climbs. It waits at the stage angle (25-30 deg
        // is clear 30-51 in) until the carriage is above the final-angle
        // height, finishes to 20 deg there (20 deg is clear 37.5-53.5 in),
        // and the carriage waits at the pre-top height until the arm is
        // strictly inside the final gate angle.
        // The station is where a carriage leaving L4 waits until the arm has
        // cleared band A (25-100 deg are all clear at 31-35 in).
        public static final double L4_STATION_HEIGHT = 33.0;        // Inches
        // Carriage height from which the arm may leave RAISE on the way to a
        // scoring pose, under the arm-side clamp (Superstructure
        // .armLimitForHeight), which holds it at the edge of any row the
        // carriage has not opened yet. 75-100 deg is clear from the base to
        // 36 in by the table, so this is not a table limit: below ~22 in the
        // claw's tail bar passes the funnel's sheet-metal lips, which the CAD
        // clearance model does not include, and at RAISE it is well behind
        // them. 24 in puts the tail bar ~2.5 in above the lips before the arm
        // moves. Lower it (to 18) only after jogging the arm to 75 deg and
        // raising the carriage slowly from 12 to 24 in with eyes on that gap -
        // every inch lower is 25 ms more lead for the arm, and at ~18 the
        // carriage no longer slows for its 35.5 in clamp at all.
        public static final double ARM_RELEASE_MIN_HEIGHT = 24.0;   // Inches
        // Arm-side clamp margins: a row counts as open once the carriage is
        // this far inside its band (the carriage's own clamp keeps it
        // RATCHET_MARGIN inside, so the two can never wait on each other), and
        // the arm stops this far short of a row that is not open.
        public static final double ARM_CLAMP_HEIGHT_MARGIN = 0.1;   // Inches
        // Under a ceiling the arm is sweeping up toward (leaving L4), a real
        // margin: that is the claw's rear against the middle-stage top tube,
        // the one contact where the robot has shown the table to be optimistic.
        public static final double ARM_CLAMP_CEILING_MARGIN_RISING = 1.0; // Inches
        // The arm is let go this long before the carriage reaches
        // ARM_RELEASE_MIN_HEIGHT (at its measured speed), and held at this
        // angle until the carriage is actually there. 92.5 deg keeps the tail
        // bar ~1.1 in behind the funnel's sheet metal (2.25 in at RAISE).
        // Set the lead to 0 to release the arm at the height instead.
        public static final double ARM_RELEASE_LEAD_SECONDS = 0.2;
        public static final double ARM_EARLY_HOLD_ANGLE = 92.5;     // Degrees
        // On an L4 climb the carriage is clamped for the rows between the arm
        // and the limit the arm has been sent to, plus this much further (two
        // table rows), so it never parks above the height that would let the
        // arm into its next row.
        public static final double ARM_COMMIT_LOOKAHEAD = 10.0;     // Degrees
        public static final double ARM_CLAMP_ANGLE_MARGIN = 2.5;    // Degrees
        // "Near the reef" (see Superstructure.setNearReefSupplier) stays true
        // this long after the last sighting: a dropped camera frame is not
        // clearance.
        public static final double NEAR_REEF_RELEASE_SECONDS = 0.4;
        // The angle the arm may hold while the carriage is still below the
        // final-angle height. 27.5 is the middle of the 25-30 row (clear
        // 30-51 in). The row's edge (25.0) is not usable: a through-bore
        // reading of 24.x there is in the 20-25 row, which is blocked below
        // 35.5 in.
        public static final double L4_STAGE_ANGLE = 27.5;           // Degrees
        public static final double L4_PRE_TOP_HEIGHT = 48.0;        // Inches
        public static final double L4_FINAL_ANGLE_MIN_HEIGHT = 37.0; // Inches: arm may finish to 20 deg
        public static final double L4_FINAL_GATE_ANGLE = 22.5;      // Degrees: carriage may go to the top
        // Return ("drop before pivoting"): the carriage descends at the L4
        // angle (20 deg is clear 37.5-53.5 in). Below the rotate-max height
        // the arm heads for RAISE in one sweep under the arm-side clamp, and
        // the carriage's target steps down as the arm rises: the drop height
        // until the arm has passed the stage-done angle, the station until it
        // has passed BAND_PASS_MIN_ANGLE, then the real target.
        public static final double L4_RETURN_DROP_HEIGHT = 39.0;         // Inches
        // Not a clearance gate, a speed-shaping waypoint: the exit's first leg
        // runs at full pace to here (still at the L4 angle, clear 35.5-52.5
        // in), so the carriage is already slowing as the arm is released, and
        // the rest of the way to the drop height and the station is paced to
        // the arm (Superstructure.l4ExitPace).
        public static final double L4_RETURN_SHAPE_HEIGHT = 41.5;        // Inches
        public static final double L4_RETURN_ROTATE_MAX_HEIGHT = 43.0;   // Inches
        public static final double L4_RETURN_STAGE_DONE_ANGLE = 40.0;    // Degrees: carriage may leave the drop height for the station

        // Kept below a ceiling (or above a floor) by the continuous carriage
        // targets, so the carriage decelerating into its clamp still lands
        // inside the corridor. The table already carries ~1 in of model margin.
        public static final double RATCHET_MARGIN = 0.5;           // Inches

        // --- Tolerances ---
        // The arm counts as "at" a gate angle within this tolerance (the
        // through bore reads the real arm, after the chain backlash).
        public static final double SAFE_ANGLE_TOLERANCE = 3.0; // Degrees
        // Final settle wait per planned move. Safety gates never time out.
        public static final double SETTLE_TIMEOUT_SECONDS = 3.0;
        // A settle also ends when both mechanisms have stopped: the closed
        // loops hold their latched setpoints, so once motion has ceased,
        // waiting longer cannot improve the pose - it only burns match time
        // when something (chain backlash, a mis-set kG) leaves a mechanism
        // resting just outside its at-target window.
        // One elevator velocity LSB is 0.55 in/s (16 ms x 2 filter), so a
        // threshold at or below that is an equals-zero test; 1.0 is about two
        // LSB, and the stopped branch is debounced as well.
        public static final double SETTLE_STOPPED_ELEVATOR_IN_S = 1.0;  // in/s
        public static final double SETTLE_STOPPED_DEBOUNCE_SECONDS = 0.08;
        public static final double SETTLE_STOPPED_PIVOT_DEG_S = 3.0;    // deg/s
        // Minimum settle dwell, so "stopped" cannot fire before the mechanisms
        // have started moving.
        public static final double SETTLE_MIN_SECONDS = 0.2;
        // Same idea for a mid-sequence "arm has arrived" wait: the S-curve
        // takes ~0.1 s to get the arm above the stopped threshold.
        public static final double ARM_ARRIVED_MIN_SECONDS = 0.15;

        // --- Table walks (Superstructure) ---
        // Resolution of the single-mechanism path checks (elevatorPathClear /
        // pivotPathClear, used by the Testing-tab moves). VERIFY they stay
        // finer than the table (rows 5 deg apart, band edges on 0.5 in), so a
        // check cannot step over a blocked band.
        public static final double PATH_CHECK_HEIGHT_STEP = 0.25; // Inches
        public static final double PATH_CHECK_ANGLE_STEP = 0.5;   // Degrees
        // The carriage clamp (ceilingForSweep / floorForSweep) samples the
        // arm's remaining sweep every SWEEP_STEP, half a table row.
        // The scoring poses sit ON row edges (L4 is 20.0 deg, L3 25.0), so an
        // arm at rest reads either side of the edge from one loop to the next
        // - and the two rows' limits differ by 2 to 5.5 in, which would send
        // the carriage back and forth between them. Where the arm IS, the walk
        // therefore takes the tighter of the rows within EDGE_WINDOW of it:
        // constant while the arm sits on the edge, and conservative.
        // A blocked row within IMMINENT_SWEEP of the arm is one the arm is
        // about to enter. A measured angle is nudged in SNAP_ANGLE_STEP steps
        // (at most SAFE_ANGLE_TOLERANCE) onto a row that holds the height
        // (snapIntoTable). VERIFY with SuperstructureCorridorTest and
        // SuperstructureSequenceSimTest after any change.
        public static final double SWEEP_STEP = 2.5;      // Degrees
        public static final double EDGE_WINDOW = 1.5;     // Degrees
        public static final double IMMINENT_SWEEP = 5.0;  // Degrees
        public static final double SNAP_ANGLE_STEP = 1.0; // Degrees

        // --- Carriage setpoint latch (Superstructure.commandCarriage) ---
        // Every new setpoint restarts the MAXMotion profile, so the latch
        // sends one only when the clamp has moved far enough. TUNE with
        // SuperstructureSequenceSimTest, which fails a move whose setpoints
        // alternate or flood the controller.
        // raiseArm puts the carriage back only if an exit moved it at least
        // this far (never a zero-length move).
        public static final double RAISE_ARM_PUT_BACK_MIN = 0.5; // Inches
        // Targets closer than this to the commanded height do not change the
        // ratchet's direction.
        public static final double DIRECTION_EPSILON = 0.01; // Inches
        // Smallest clamp advance worth a profile restart while the carriage is
        // moving. As the arm sweeps, the clamp climbs the table one row at a
        // time (1-1.5 in every 20-40 ms); sending each stair would give the L4
        // climb some fifteen setpoints, a dozen of them within 0.4 s, every one
        // a profile restart - a visibly jumpy climb. Waiting for a run worth
        // having makes it four or five.
        public static final double LATCH_MIN_STEP = 4.0; // Inches
        // A carriage at rest waits rather than start a run to a clamp shorter
        // than this (see commandCarriage).
        public static final double LATCH_MIN_RUN = 8.0; // Inches
        // Below this speed the carriage is waiting on the arm: any real
        // advance of at least LATCH_CRAWL_MIN_STEP is sent, so it keeps moving.
        public static final double LATCH_CRAWL_SPEED = 3.0;    // in/s
        public static final double LATCH_CRAWL_MIN_STEP = 0.5; // Inches
        // A clamp that moves back by less than this is ignored (arm-angle
        // chatter at a gate).
        public static final double LATCH_RETREAT_MIN = 0.25; // Inches
        // A carriage that is past its clamp by no more than this comes back to
        // the clamp; further than that it stops. Coming back matters: the
        // arm-side clamp only opens a row once the carriage is inside it, so a
        // carriage that arrived half an inch high and held there would leave
        // each mechanism waiting for the other.
        public static final double LATCH_RETREAT_MAX = 1.5; // Inches
        // L4 exit: the carriage is re-targeted from L4_RETURN_SHAPE_HEIGHT to
        // the drop height, and paced, this far before it reaches the shaping
        // height, so it never stops at that waypoint. TUNE with
        // SuperstructureSequenceSimTest.pacesCompared.
        public static final double L4_RETURN_SHAPE_WINDOW = 1.0; // Inches

        // --- Carriage pace per move (ElevatorConstants.PACE_*) ---
        // The arm is the long pole of every move into or out of an upper pose;
        // at full pace the carriage reaches each clearance limit before the arm
        // has opened it, brakes, and is released again. TUNE with
        // SuperstructureSequenceSimTest.pacesCompared (build/sim/paces.csv):
        // each is the fastest pace at which the carriage never has to slow
        // for the arm.
        // (Superstructure copies them into fields that test varies.)
        // L4 climb: BRISK. With the arm's 0.2 s release lead the sim runs it
        // hitch-free even at FULL (1.66 s), but barely - the carriage reaches
        // its 35.5 in clamp as the arm opens it - and a real arm lags its
        // profile; BRISK (1.96 s) has ~0.15 s in hand and never drops below
        // cruise.
        public static final Pace CLIMB_TO_L4_PACE = Pace.BRISK;
        // L3 climb: FULL - the carriage has no clamp to wait at once the roof
        // has opened.
        public static final Pace CLIMB_TO_L3_PACE = Pace.FULL;
        // L4 exit, from the shaping height to the station: SLOW, so the
        // carriage is still moving when the arm opens the drop height and the
        // station (2.02 s, never under ~7 in/s; at FULL it is 2.00 s with two
        // dead stops).
        public static final Pace L4_EXIT_PACE = Pace.SLOW;
        // From an algae pose the arm has 140 deg to come round before L4.
        public static final Pace ALGAE_TO_L4_PACE = Pace.EASY;

        // --- Score gate (Superstructure.readyToScore) ---
        // How close to the preset the measured pose must be, and how still
        // both mechanisms must be, before a score is released. TUNE.
        public static final double SCORE_READY_HEIGHT_TOL = 0.75;        // Inches
        public static final double SCORE_READY_ANGLE_TOL = 3.0;          // Degrees
        public static final double SCORE_READY_MAX_ELEVATOR_SPEED = 2.0; // in/s
        public static final double SCORE_READY_MAX_PIVOT_SPEED = 15.0;   // deg/s
        // "Landed": once both mechanisms have been still this long the pose is
        // as good as it is going to get, and a wider angle window counts. The
        // loop closes on the rotor and the chain has slack, so the through
        // bore can come to rest a few degrees past the target; CorAl's landing
        // correction normally pulls it back in, but the score button must
        // NEVER be locked out by it - with only the tight window, the one way
        // left to score would be by hand. The height gate is NOT relaxed: above
        // the 48 in pre-top height the carriage only gets there with the arm
        // inside 22.5 deg.
        public static final double SCORE_READY_LANDED_ANGLE_TOL = 8.0; // Degrees
        public static final double SCORE_READY_LANDED_SECONDS = 0.6;   // Seconds

        // --- Eject and back-off (Superstructure.score, ejectCoral, scoreAlgae) ---
        // A coral eject runs the rollers at least CORAL_EJECT_SECONDS and then
        // until the CANrange no longer sees the coral, for at most
        // CORAL_EJECT_CLEAR_TIMEOUT_SECONDS more; still detected, the score
        // keeps the pose for a second try. An algae eject is timed only. TUNE.
        public static final double CORAL_EJECT_SECONDS = 0.5;               // Seconds
        public static final double CORAL_EJECT_CLEAR_TIMEOUT_SECONDS = 1.0; // Seconds
        public static final double ALGAE_EJECT_SECONDS = 0.5;               // Seconds
        // Distance the drivetrain must travel from where it ejected before an
        // L3/L4 exit may start. Leaving those poses swings the claw through
        // 75-110 deg, where it reaches 9-12 in (8.9-11.7 in the CAD) past the
        // front bumper face - into the reef if the robot is still flush
        // against it. VERIFY against the claw's reach if the arm changes.
        public static final double REEF_BACKOFF_METERS = 0.35;
    }

    // CANdle disabled (Sept 2026): there is no CANdle on the robot, so the LED
    // subsystem (subsystems/LEDs.java) and its constants are commented out.
    // To restore: uncomment this block, the two imports marked "CANdle
    // disabled" at the top of this file, every line of LEDs.java, and the
    // lines marked "CANdle disabled" in RobotContainer.java and Dashboard.java.
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
    // 	// GRB is the byte order of common WS2812 / NeoPixel strips. VERIFY on
    // 	// the installed strip: change it if colors appear swapped.
    // 	public static final StripTypeValue STRIP_TYPE = StripTypeValue.GRB;
    //
    // 	// --- Timing (seconds) ---
    // 	public static final double ENDGAME_WARNING_TIME = 20.0; // Teleop time remaining when the endgame pattern starts
    //
    // 	// --- Colors (RGBW, 0-255; the white channel is 0 for standard RGB strips; TUNE) ---
    // 	public static final RGBWColor RED_ALLIANCE_COLOR = new RGBWColor(255, 0, 0);
    // 	public static final RGBWColor BLUE_ALLIANCE_COLOR = new RGBWColor(0, 0, 255);
    // 	public static final RGBWColor NO_ALLIANCE_COLOR = new RGBWColor(120, 0, 255); // Purple: the alliance is not known yet
    // 	public static final RGBWColor GAME_PIECE_COLOR = new RGBWColor(0, 255, 0);    // Green: game piece held
    // 	public static final RGBWColor TRACKING_COLOR = new RGBWColor(0, 200, 255);    // Cyan: vision tracking
    // 	public static final RGBWColor ENDGAME_COLOR = new RGBWColor(255, 180, 0);     // Yellow: endgame
    //
    // 	// --- Animations (frames per second; Larson size in LEDs; TUNE) ---
    // 	public static final int STROBE_FRAME_RATE = 8;
    // 	public static final int LARSON_SIZE = 6;
    // 	public static final int LARSON_FRAME_RATE = 30;
    // 	public static final int RAINBOW_FRAME_RATE = 40;
    // }

    /**
     * Constants for the driver and operator controllers (RobotContainer).
     */
    public static final class ControlsConstants {
        // --- Driver Station USB ports (VERIFY in the Driver Station's USB tab) ---
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Driver trigger travel (0-1) that starts a hold-to-align. TUNE.
        public static final double ALIGN_TRIGGER_THRESHOLD = 0.3;
        // Operator trigger travel (0-1) that counts as a pull: LT (HOME), RT
        // (score) and, under the manual take-over, LT / RT (rollers in / out).
        // 0.5 is WPILib's default. TUNE with the operator.
        public static final double OPERATOR_TRIGGER_THRESHOLD = 0.5;
        // Operator Back / Start must be held this long, disabled only, to zero
        // the elevator / the pivot, so a brushed button zeroes nothing. TUNE.
        public static final double ZERO_HOLD_SECONDS = 1.0; // Seconds

        // --- Rumble cues (strength 0-1, seconds; TUNE with the drive team) ---
        // Coral acquired: both controllers, one long buzz (leave the station).
        public static final double RUMBLE_ACQUIRED_STRENGTH = 1.0;
        public static final double RUMBLE_ACQUIRED_SECONDS = 0.4;
        // Pose reached: operator, short buzzes (the score trigger is live).
        public static final int RUMBLE_POSE_READY_COUNT = 2;
        public static final double RUMBLE_POSE_READY_STRENGTH = 0.8;
        public static final double RUMBLE_POSE_READY_ON_SECONDS = 0.12;
        public static final double RUMBLE_POSE_READY_OFF_SECONDS = 0.10;
        // Aligned and pose reached: driver, steady light buzz (fire).
        public static final double RUMBLE_ALIGNED_STRENGTH = 0.5;
        // Scored at L3 / L4 and waiting for room, or a move waiting for the
        // reef to be clear: driver, slow pulses, repeated (back away).
        public static final int RUMBLE_BACK_AWAY_COUNT = 2;
        public static final double RUMBLE_BACK_AWAY_STRENGTH = 0.6;
        public static final double RUMBLE_BACK_AWAY_ON_SECONDS = 0.15;
        public static final double RUMBLE_BACK_AWAY_OFF_SECONDS = 0.35;
        // Score pulled with nothing to score from: operator, one tick.
        public static final double RUMBLE_NO_SCORE_STRENGTH = 0.4;
        public static final double RUMBLE_NO_SCORE_SECONDS = 0.08;
    }

    /**
     * Constants for teleop driving (RobotContainer) and the drivetrain's
     * SysId routines and desktop simulation (Swerve).
     */
    public static final class DriveConstants {
        // Fraction of the drivetrain's theoretical top speed (and top
        // rotation rate) the driver sticks command at full deflection. This
        // is the default for the "Drive - Teleop Speed Scale" tunable, which
        // can be changed from the dashboard without a redeploy: 0.25 for
        // indoor testing, raise toward 1.0 for competition driving.
        public static final double TELEOP_SPEED_SCALE = 0.25;

        // Rotation rate at full stick before the speed scale: 3/4 rotation
        // per second. TUNE with the drivers.
        public static final double MAX_ANGULAR_RATE_ROT_PER_SEC = 0.75; // Rotations per second

        // Stick deadband as a fraction of the scaled top speed (so it stays
        // 20% of stick travel at every speed scale).
        public static final double STICK_DEADBAND = 0.2;

        // --- Speed scaling by carriage height ---
        // Above SLOW_START_HEIGHT the driver's speed (and rotation rate) is
        // scaled down linearly, reaching SLOW_MIN_SCALE at SLOW_FULL_HEIGHT
        // and above. TUNE with the drivers.
        public static final double SLOW_START_HEIGHT = 16.5; // Inches (the low box roof)
        public static final double SLOW_FULL_HEIGHT = 45.0;  // Inches
        public static final double SLOW_MIN_SCALE = 0.40;    // Fraction of the scaled speed

        // D-pad nudges: robot-centric, in all eight directions. TUNE.
        public static final double NUDGE_SPEED = 0.5; // m/s

        // How the modules drive the teleop sticks and the nudges. Closed-loop
        // velocity: every module tracks the requested ground speed regardless
        // of battery sag, and teleop feels the same as autonomous path
        // following. OpenLoopVoltage is the alternative (the sticks set power,
        // so the robot slows as the battery sags). TUNE with the drivers.
        public static final DriveRequestType TELEOP_DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;

        // --- SysId (Test mode only; logged by SignalLogger) ---
        // TUNE if a test browns out the robot or barely moves it.
        // Translation: dynamic step reduced to 4 V to prevent a brownout.
        public static final double SYSID_TRANSLATION_STEP_VOLTS = 4; // Volts
        public static final double SYSID_STEER_STEP_VOLTS = 7;       // Volts
        // Rotation runs in rad/s, which SysId only accepts as "volts": the
        // ramp is rad/s per second (passed as V/s), the step rad/s (as V).
        public static final double SYSID_ROTATION_RAMP_RATE = Math.PI / 6; // rad/s per second
        public static final double SYSID_ROTATION_STEP = Math.PI;          // rad/s

        // Desktop simulation: the swerve sim thread runs at this period,
        // faster than the 20 ms robot loop, so the PID gains behave more
        // reasonably. Affects the sim only.
        public static final double SIM_LOOP_PERIOD_SECONDS = 0.005; // Seconds (5 ms)
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

        // How the modules drive the path follower's commands. Closed-loop
        // velocity: PathPlanner's feedforward is a ground speed, and each
        // module reaches it regardless of battery voltage, which the gains
        // above assume. VERIFY the paths again after any change.
        public static final DriveRequestType PATH_DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;

        // Named-command timeout: how long an auto waits at a coral station for
        // the CANrange to confirm a coral before moving on
        public static final double AUTO_INTAKE_TIMEOUT_SECONDS = 3.0;
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
        // this many. While the robot is disabled (pit, queue, between
        // periods - most of the camera's powered-on life) full-rate AprilTag
        // processing is wasted work that only heats the camera and spins the
        // fan; Limelight's guidance is 100-200. Full rate (0) is restored
        // the instant the robot enables, so tracking performance is
        // unaffected. At ~90 fps, 100 still yields ~1 solve/s while disabled,
        // plenty for the pre-match heading seed. Selecting Test mode on the
        // Driver Station also lifts the throttle while disabled, so the
        // dashboard's vision readouts are live on the bench.
        public static final int DISABLED_THROTTLE = 100;
        public static final int ENABLED_THROTTLE = 0;
        // The dashboard's seen-tag readouts ("Vision/Best Tag", "Vision/Best
        // Tag Camera", "Vision/Visible Tags") ignore a camera whose tag list
        // has not changed for this long (NetworkTables keeps the last value
        // of a camera that lost power or its link). Longer than the gap
        // between solves at the disabled throttle.
        public static final double SEEN_TAG_STALE_SECONDS = 10.0;

        // --- Camera Roles ---
        // Which tag class each Limelight may supply to the alignment tracker,
        // same order as LIMELIGHT_NAMES (every camera still feeds MegaTag pose
        // estimation with whatever tags it sees). The rear funnel camera is
        // the only one that should ever align on a coral station. The barge
        // camera is assigned the barge and processor tags, but its pose is an
        // unmeasured placeholder, so until that is measured it supplies
        // nothing and barge / processor alignment has no target.
        public enum TagClass { REEF, CORAL_STATION, BARGE, PROCESSOR, NONE }
        public static final TagClass[][] LIMELIGHT_TRACKING_CLASSES = {
            {TagClass.CORAL_STATION},              // funnel - rear-facing, watches the coral station behind the robot
            // barge - front, high: the only camera that can see the barge and
            // processor tags with the robot facing them (the reef camera is
            // pitched 20 deg down at 16 in - a 51 in-high tag only enters its
            // view beyond 6 m). It supplies nothing until its pose is measured:
            // alignment converts the tag into the robot frame with the lens
            // pose, and a placeholder pose would center the robot on the wrong
            // spot. Fill in LIMELIGHT_POSES[1] and set measured = true.
            {TagClass.BARGE, TagClass.PROCESSOR},
            {TagClass.REEF},                       // reef   - front-left, yawed toward the centerline
        };
        // --- Camera Poses in Robot Space ---
        // Where each lens sits on the robot. MegaTag uses this to convert what
        // a camera sees into where the robot is, so an error here shifts every
        // fused pose from that camera. Pushed to the camera at startup (so it
        // lives in version control and survives a camera reset), but only for
        // cameras marked measured - unmeasured cameras keep whatever their
        // web UI holds.
        //
        // Limelight robot-space convention (docs "3D Coordinate Systems"):
        // origin at the frame center projected to the floor; X+ forward,
        // Y+ toward the robot's right (note: opposite of WPILib's +Y = left),
        // Z+ up; meters and degrees. Pitch is entered as positive = lens
        // tilted up, yaw as the heading of the lens (180 = rear-facing).
        // When entering a new pose, VERIFY the pitch/yaw signs against the
        // camera's web-UI 3D preview (it mirrors these values) - if the
        // preview shows the camera pointing the wrong way, flip the sign here.
        public static final class CameraPose {
            public final double forwardMeters, sideMeters, upMeters;
            public final double rollDegrees, pitchDegrees, yawDegrees;
            /** False = placeholder: the pose is not pushed to the camera, and the camera supplies no alignment target or heading seed. */
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

            /**
             * The lens position in the WPILib robot frame (+X forward,
             * +Y left): Limelight's side axis is positive to the right.
             */
            public Translation2d lensOnRobot() {
                return new Translation2d(forwardMeters, -sideMeters);
            }
        }

        // Same order as LIMELIGHT_NAMES.
        public static final CameraPose[] LIMELIGHT_POSES = {
            // funnel (rear): lens 1.0 in inside the back edge of the 30 in
            // frame -> 14.0 in behind center; centered side-to-side;
            // 29.625 in above the floor; facing straight back and tilted
            // 50 deg above horizontal ("50 degrees north of west" in the
            // side view with west = rearward, north = up) so the coral
            // station tag stays in frame when the rear bumpers are flush.
            new CameraPose(-14.0 * 0.0254, 0.0, 29.625 * 0.0254, 0.0, 50.0, 180.0, true),
            // barge: PLACEHOLDER, NOT MEASURED - the CAD has its 1x1 mount
            // (front-left, ~40.6 in up, 13.4 in forward) but no camera body
            // in it, so there is nothing to derive a pose from. Measure the
            // lens pose on the robot, enter it here and set measured = true;
            // until then this camera supplies no alignment target and no
            // heading seed.
            new CameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
            // reef (front-left, base stage): from the CAD (Leviathan STEP,
            // lens-barrel cylinder of the LL3G model in the "2x1
            // Perpendicular Angled Limelight Mount"): lens 11.04 in forward
            // of center, 11.25 in left of center (negative in Limelight's
            // Y-right convention), 15.78 in above the floor; optical axis
            // pitched 20 deg down and yawed 30 deg toward the robot's
            // centerline (to the right). Reef alignment runs on the robot
            // with this pose. To re-check after a remount, open the camera
            // web UI 3D preview (http://limelight-reef.local:5801): the
            // model should sit front-left, looking forward-right and down;
            // if it looks left instead, flip the yaw sign.
            new CameraPose(11.04 * 0.0254, -11.25 * 0.0254, 15.78 * 0.0254, 0.0, -20.0, -30.0, true),
        };

        // --- Field (2025 Reefscape) ---
        // The AprilTag layout the alignment heading and the pose plausibility
        // gate use, and the field size PathPlanner's alliance flip must use.
        // Welded is the standard event field; switch to
        // k2025ReefscapeAndyMark for an AndyMark field.
        public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;
        // A fused pose may lie this far outside the field before it is rejected
        public static final double FIELD_BOUNDS_MARGIN_METERS = 0.5;

        // --- Pose-estimate plausibility gates ---
        // Single-tag MegaTag1 solves (the heading seed) are only trusted when
        // unambiguous and close; these are Limelight's documented thresholds.
        public static final double MT1_SINGLE_TAG_MAX_AMBIGUITY = 0.7;
        public static final double MT1_SINGLE_TAG_MAX_DISTANCE_METERS = 3.0;
        // Tag count from which a MegaTag1 solve is a strong heading seed: its
        // heading is fused with MT1_MULTI_TAG_ROTATION_STD_DEV, and it may set
        // the auto-start heading (Swerve.resetPoseForAuto, with the
        // HEADING_SEED_* rules) and the driver's forward before the first
        // driver zero. A single-tag solve carries the pose-flip ambiguity.
        // TUNE: 3 trusts only solves with a wider tag spread.
        public static final int MT1_STRONG_SEED_MIN_TAGS = 2;
        // Heading std devs (rad) for MegaTag1 while seeding. The estimator only
        // closes part of the heading error per fused solve (about 2/3 at 0.05
        // against its 0.1 rad state std dev), and the disabled throttle yields
        // ~1 solve/s, so strong (2+ tag) solves are trusted tightly to converge
        // in a few seconds; fewer tags (a lone tag is already gated on
        // ambiguity/distance) less so.
        public static final double MT1_MULTI_TAG_ROTATION_STD_DEV = 0.05;
        public static final double MT1_SINGLE_TAG_ROTATION_STD_DEV = 0.3;
        // MegaTag2 translations from distant tags add little and can jump
        public static final double MT2_MAX_AVG_TAG_DISTANCE_METERS = 6.0;
        // How recently a trusted MegaTag1 solve must have been fused for the
        // heading to count as field-referenced (auto start then keeps it
        // instead of the path's nominal heading), and how far it may disagree
        // with the nominal heading before the nominal one wins.
        public static final double HEADING_SEED_FRESHNESS_SECONDS = 3.0;
        public static final double HEADING_SEED_MAX_DISAGREEMENT_DEGREES = 20.0;
        // ...and the estimator heading must have converged on that solve: it
        // must agree with the solve's own heading within this much
        public static final double HEADING_SEED_AGREEMENT_DEGREES = 3.0;
        // Heading re-seed while enabled (driver Y, Vision.requestHeadingReseed):
        // MegaTag1 is fused for this long so the pose heading corrects from
        // tag geometry
        public static final double HEADING_RESEED_WINDOW_SECONDS = 2.0;
        // Spin rejection (Swerve): vision poses are dropped while the robot
        // turns faster than this, and for a short time after the spin ends
        // (motion blur and rolling shutter corrupt the solve, and the image
        // was captured 25-100 ms before it arrives, so the last smeared
        // frames come in after the spin). TUNE.
        public static final double VISION_MAX_OMEGA_RAD_PER_SEC = 2.0;     // rad/s
        public static final double VISION_REJECT_AFTER_SPIN_SECONDS = 0.2; // Seconds

        // --- Pose-estimate standard deviations (Vision.updateRobotPosition) ---
        // Translation std dev = XY_STD_DEV_BASE + XY_STD_DEV_PER_DISTANCE_SQUARED
        // x (average tag distance)^2 / tag count: confidence grows with tag
        // count and closeness. MegaTag2's heading is the robot's own gyro
        // echoed back, so its rotation std dev is effectively infinite. TUNE.
        public static final double XY_STD_DEV_BASE = 0.3;                 // Meters
        public static final double XY_STD_DEV_PER_DISTANCE_SQUARED = 0.4; // Meters per m^2 of average tag distance
        public static final double MT2_ROTATION_STD_DEV = 9999999;        // Radians (heading ignored)
        // The Field widget draws each camera's last fused pose for this long.
        public static final double FUSED_POSE_DISPLAY_SECONDS = 1.0; // Seconds

        // --- Tag Classes (2025 Reefscape field) ---
        // Which tag IDs belong to each alignment class. A tag in none of these
        // lists has class NONE and the alignment tracker ignores it. (All
        // tags still feed MegaTag.) Barge and processor tags are classified,
        // but no camera supplies them until the barge camera's pose is
        // measured (see LIMELIGHT_TRACKING_CLASSES).
        public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        public static final int[] CORAL_STATION_TAGS = {1, 2, 12, 13};
        public static final int[] BARGE_TAGS = {4, 5, 14, 15};
        public static final int[] PROCESSOR_TAGS = {3, 16};

        // --- Tracking Goal Distances (meters, robot frame) ---
        // Where the tag should sit relative to the robot center when in
        // position; the tracker works in the robot frame, so these are
        // geometry, not camera readings. Flush = the tag's face at the bumper
        // face: half the bumper-to-bumper length (36.5 in / 2 = 0.464 m) plus
        // a little standoff. TUNE by pushing the robot into position and
        // copying the Vision/Distance (forward) reading.
        public static final double REEF_FLUSH_DISTANCE = 0.47;    // Front bumpers flush with the reef base (L2-L4 + algae)
        public static final double L1_SCORE_DISTANCE = 1.0;       // Standoff for L1 so the CorAl can swing to 100 deg without hitting
        public static final double STATION_FLUSH_DISTANCE = 0.47; // Rear bumpers flush with the coral station wall (tag behind the robot)
        // L3 / L4 alignment holds this far back from flush until the scoring
        // pose is reached (and backs out to it after the score). The claw
        // reaches 0.29 m past the front bumper at RAISE, so 0.40 m leaves
        // ~0.11 m to the reef face, and it is more than the 0.35 m of departure
        // the automatic stow waits for.
        public static final double REEF_STANDOFF_EXTRA = 0.40;    // Meters back from flush
        // "A reef face is right in front of the bumper": a reef tag closer than
        // this (robot center to tag, forward) and roughly ahead. Flush is 0.47;
        // the standoff position (0.87) must read as clear.
        public static final double NEAR_REEF_DISTANCE = 0.80;     // Meters
        public static final double NEAR_REEF_LATERAL = 0.60;      // Meters either side
        // FIRST GUESSES: barge / processor alignment has no target until the
        // barge camera's pose is measured. Once it is, TUNE both on the
        // field (push the robot into position, copy Vision/Distance):
        public static final double BARGE_SCORE_DISTANCE = 1.2;    // Centered on the barge tag, this far back, square - the net shot
        public static final double PROCESSOR_DISTANCE = 0.55;     // Centered on the processor tag, front bumper just off the wall

        // --- Reef Branch Alignment (meters, robot-frame Y) ---
        // Each reef face has two scoring branches, one either side of the tag,
        // 13 in apart (game manual) -> 6.5 in from the tag center. Centering
        // on the left branch puts the tag this far to the robot's right. TUNE
        // on the field (Vision/Lateral with the robot centered on a branch).
        public static final double REEF_BRANCH_OFFSET = 0.165;

        // A camera-space Z (forward) below this is not a real tag solve (an
        // empty or zeroed targetpose array)
        public static final double MIN_CAMERA_Z = 0.1;

        // --- Tracking Gains and Limits ---
        public static final class TrackingGains {
            public static final double DISTANCE_kP = 1.5;  // TUNE - m/s of drive per meter of position error
            public static final double ROTATION_kP = 0.06; // TUNE - rad/s of rotation per degree of heading error

            // Deadbands: errors below these are treated as zero. Lateral is
            // tight because a coral on a branch has only ~3 cm of clearance
            // per side; fore/aft is absorbed by the reef base when flush.
            public static final double FORWARD_ERROR_DEADBAND = 0.03;  // Meters
            public static final double LATERAL_ERROR_DEADBAND = 0.015; // Meters
            public static final double ROTATION_ERROR_DEADBAND = 1.0;  // Degrees of heading error

            // Smallest drive command outside a deadband: a P command below
            // this would not overcome static friction and the robot would
            // stall just outside the deadband
            public static final double MIN_LINEAR_VELOCITY = 0.12; // m/s
            public static final double MAX_LINEAR_VELOCITY = 2.0;  // m/s command clamp while tracking
            public static final double MAX_ANGULAR_VELOCITY = 1.0; // rad/s command clamp while tracking
            // How the modules drive the tracking commands. Closed-loop
            // velocity: open loop, the small commands the servo produces near
            // its deadbands would never overcome static friction. VERIFY the
            // alignment again after any change.
            public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;

            // Commands are slew-limited: an unlimited P output steps (0 to
            // 2 m/s on the first loop, and to zero the instant a frame is
            // missed), which is wheel slip and a lurch. 3 m/s^2 is gentle enough
            // with the elevator raised; the P law itself never asks for more
            // than kP x speed of deceleration (1.5 x 2 = 3 m/s^2), so the limit
            // does not cause overshoot on the way in.
            public static final double MAX_LINEAR_ACCELERATION = 3.0;  // m/s^2
            public static final double MAX_ANGULAR_ACCELERATION = 6.0; // rad/s^2
            // The slew limiter's time step is clamped to this range, so a loop
            // overrun (or the first loop of a new command) cannot let a step
            // through. TUNE.
            public static final double SLEW_MIN_DT_SECONDS = 0.005; // Seconds
            public static final double SLEW_MAX_DT_SECONDS = 0.05;  // Seconds

            // Once inside the deadbands the robot holds still until an error
            // grows past deadband x this. Without the gap, an error sitting ON
            // a deadband edge toggles the command every loop.
            public static final double DEADBAND_EXIT_RATIO = 1.6;

            // Filter on the latched tag's field position (Vision class note):
            // fraction of each new camera frame blended in. A sample further
            // than the outlier distance from the estimate is ignored unless it
            // persists for that many frames.
            public static final double TARGET_FILTER_ALPHA = 0.3;
            public static final double TARGET_OUTLIER_METERS = 0.25;
            public static final int TARGET_OUTLIER_FRAMES = 3;
            // Same idea for the offset between the field-true heading (from
            // each frame's MegaTag1 solve) and the pose estimator's heading.
            // Slower, with a wider gate that must persist longer: a single-tag
            // solve's heading is noisier than its position and can flip.
            public static final double HEADING_FILTER_ALPHA = 0.2;
            public static final double HEADING_OUTLIER_DEGREES = 12.0;
            public static final int HEADING_OUTLIER_FRAMES = 5;

            // Flush means the bumper is ON the reef or the wall, and a
            // closed-loop velocity command into a wall is a stalled drivetrain
            // stuttering against it. If the robot is being told to move, is not
            // moving, is laterally in position and is within this much of the
            // forward goal for this long, it has arrived by contact.
            public static final double CONTACT_FORWARD_ERROR = 0.10; // Meters
            public static final double CONTACT_MAX_SPEED = 0.04;     // m/s measured
            public static final double CONTACT_SECONDS = 0.3;
            // "Told to move" = the slewed command is at least this fraction of
            // MIN_LINEAR_VELOCITY: a little under the floor, so a command
            // resting on the floor always counts. TUNE.
            public static final double CONTACT_MIN_COMMAND_FRACTION = 0.9;

            // The tracker keeps its first tag, and while that tag is out of
            // view carries its last sighting on odometry, for this long: the
            // goal cannot flip between adjacent reef faces, and an approach
            // whose tag leaves the camera's view at the end (the rear funnel
            // camera loses the station tag ~0.55 m from the wall) still
            // finishes on dead reckoning.
            public static final double TARGET_MEMORY_SECONDS = 1.5;
        }
    }

    /**
     * Constants for the Elastic dashboard (Dashboard): the Testing tab's
     * defaults and the alert thresholds that are not a mechanism's own.
     */
    public static final class DashboardConstants {
        // Roller duty cycle the Testing tab's Intake Speed slider starts at
        // (seeded only while no value is stored). TUNE.
        public static final double TEST_INTAKE_SPEED_DEFAULT = 0.1; // Duty cycle, -1 to 1
        // The low-battery alert fires while disabled below this resting
        // voltage (sags under load in a match are normal). VERIFY against the
        // team's battery beak readings.
        public static final double LOW_BATTERY_VOLTS = 12.0; // Volts
    }

    /**
     * Constants for the dashboard tunables (util/Tunables.java): the factory
     * defaults version, the poll period, and the clamp range of every
     * numeric tunable. The defaults themselves live with their mechanism
     * above.
     */
    public static final class TunablesConstants {
        // Version stamp of the factory defaults in Constants. Stored values
        // survive deploys, so changing a default does nothing on a robot that
        // already has the key stored - unless this number is bumped. On the
        // first boot after a bump, Tunables.init() does one of two things:
        //   - if it has a targeted migration block for the stored version, it
        //     overwrites only the keys named there and keeps everything else
        //     the team has tuned on the dashboard;
        //   - otherwise it overwrites every tunable with the new defaults.
        // Bump it when a default changes and must take effect on the robot
        // (and add a migration block when only a few defaults moved); leave it
        // alone to preserve values tuned on the dashboard.
        // Version 15 is the set of defaults as released (Sept 2026); earlier
        // versions were pre-release tuning rounds.
        public static final int DEFAULTS_VERSION = 15;

        // The Elevator and the CorAl check the tunables this often while
        // disabled, and re-apply an edit to their controllers. TUNE.
        public static final double TUNABLE_POLL_SECONDS = 0.5; // Seconds (twice a second)

        // --- Clamp ranges ---
        // Every numeric getter clamps its live value to [MIN, MAX], so a typo
        // on the dashboard cannot command something dangerous (the tracking
        // gains' ceiling is a multiple of their default, below; the two on/off
        // tunables read as on at 0.5 or more and need no range). VERIFY that
        // a new default lies inside its range.
        // Drive: can neither disable driving nor exceed the drivetrain.
        public static final double TELEOP_SPEED_SCALE_MIN = 0.05;          // Fraction
        public static final double TELEOP_SPEED_SCALE_MAX = 1.0;           // Fraction
        // Elevator calibration: a typo cannot scale the encoder by more than
        // ~3x either way; a hard-stop height beyond a few inches is a
        // measurement error.
        public static final double ELEVATOR_TRAVEL_RATIO_MIN = 0.33;       // Measured / modeled
        public static final double ELEVATOR_TRAVEL_RATIO_MAX = 3.0;        // Measured / modeled
        public static final double ELEVATOR_ZERO_HEIGHT_MIN = 0.0;         // Inches
        public static final double ELEVATOR_ZERO_HEIGHT_MAX = 6.0;         // Inches
        // Elevator gains and profile. kA's ceiling is about three times the
        // CAD model: more would mean the carriage weighs far more than the
        // CAD says.
        public static final double ELEVATOR_KP_MIN = 0.0;                  // Duty per inch
        public static final double ELEVATOR_KP_MAX = 2.0;                  // Duty per inch
        public static final double ELEVATOR_KS_MIN = 0.0;                  // Volts
        public static final double ELEVATOR_KS_MAX = 3.0;                  // Volts
        public static final double ELEVATOR_KV_SCALE_MIN = 0.0;            // x free-speed model
        public static final double ELEVATOR_KV_SCALE_MAX = 2.0;            // x free-speed model
        public static final double ELEVATOR_KA_MIN = 0.0;                  // V per in/s^2
        public static final double ELEVATOR_KA_MAX = 0.012;                // V per in/s^2
        public static final double ELEVATOR_KG_MIN = 0.0;                  // Volts
        public static final double ELEVATOR_KG_MAX = 3.0;                  // Volts
        public static final double ELEVATOR_CRUISE_VELOCITY_MIN = 0.5;     // in/s
        public static final double ELEVATOR_CRUISE_VELOCITY_MAX = 60.0;    // in/s
        public static final double ELEVATOR_MAX_ACCELERATION_MIN = 1.0;    // in/s^2
        public static final double ELEVATOR_MAX_ACCELERATION_MAX = 500.0;  // in/s^2
        public static final double ELEVATOR_PROFILE_ERROR_MIN = 0.05;      // Inches
        public static final double ELEVATOR_PROFILE_ERROR_MAX = 5.0;       // Inches
        // Pivot profile (cruise below the ~550 deg/s free speed) and gravity.
        // Phoenix clamps the gravity offset to +/-0.25 rot silently, hence the
        // 1-179 deg balance angle; kG's ceiling is about three times the CAD
        // estimate (~0.3 V). Jerk 0 disables the limit (a plain trapezoid).
        public static final double PIVOT_CRUISE_VELOCITY_MIN = 10.0;       // deg/s
        public static final double PIVOT_CRUISE_VELOCITY_MAX = 500.0;      // deg/s
        public static final double PIVOT_MAX_ACCELERATION_MIN = 20.0;      // deg/s^2
        public static final double PIVOT_MAX_ACCELERATION_MAX = 3000.0;    // deg/s^2
        public static final double PIVOT_MAX_JERK_MIN = 0.0;               // deg/s^3
        public static final double PIVOT_MAX_JERK_MAX = 50000.0;           // deg/s^3
        public static final double PIVOT_KG_MIN = 0.0;                     // Volts
        public static final double PIVOT_KG_MAX = 1.0;                     // Volts
        public static final double PIVOT_BALANCE_ANGLE_MIN = 1.0;          // Degrees
        public static final double PIVOT_BALANCE_ANGLE_MAX = 179.0;        // Degrees
        // Coral detection (CANrange)
        public static final double CORAL_DETECT_DISTANCE_MIN = 0.02;       // Meters
        public static final double CORAL_DETECT_DISTANCE_MAX = 4.0;        // Meters
        public static final double CORAL_DETECT_HYSTERESIS_MIN = 0.0;      // Meters
        public static final double CORAL_DETECT_HYSTERESIS_MAX = 0.1;      // Meters
        public static final double CORAL_MIN_SIGNAL_STRENGTH_MIN = 0.0;    // Unitless
        public static final double CORAL_MIN_SIGNAL_STRENGTH_MAX = 30000.0; // Unitless
        // Vision goals (by magnitude: a pasted negative Vision/Distance cannot
        // invert a goal)
        public static final double REEF_FLUSH_DISTANCE_MIN = 0.3;          // Meters
        public static final double REEF_FLUSH_DISTANCE_MAX = 2.0;          // Meters
        public static final double STATION_FLUSH_DISTANCE_MIN = 0.3;       // Meters
        public static final double STATION_FLUSH_DISTANCE_MAX = 2.0;       // Meters
        public static final double L1_SCORE_DISTANCE_MIN = 0.3;            // Meters
        public static final double L1_SCORE_DISTANCE_MAX = 3.0;            // Meters
        public static final double BARGE_SCORE_DISTANCE_MIN = 0.3;         // Meters
        public static final double BARGE_SCORE_DISTANCE_MAX = 3.0;         // Meters
        public static final double PROCESSOR_DISTANCE_MIN = 0.3;           // Meters
        public static final double PROCESSOR_DISTANCE_MAX = 2.0;           // Meters
        public static final double REEF_BRANCH_OFFSET_MIN = 0.0;           // Meters
        public static final double REEF_BRANCH_OFFSET_MAX = 0.5;           // Meters
        // Tracking gains: 0 to this multiple of the default, so a mistyped
        // value cannot invert or destabilize the servo.
        public static final double TRACKING_DISTANCE_KP_MIN = 0.0;         // m/s per m
        public static final double TRACKING_ROTATION_KP_MIN = 0.0;         // rad/s per deg
        public static final double TRACKING_KP_MAX_MULTIPLE = 3.0;         // x the default
    }

    /**
     * Constants for AdvantageKit logging and the AdvantageScope / Glass
     * mechanism visualizations.
     */
    public static final class LoggingConstants {
        // When no USB stick is mounted the .wpilog files go to the roboRIO's
        // internal storage: keep at least this much free by deleting the oldest
        // logs (never the newest few), so a full disk cannot break Preferences.
        public static final long INTERNAL_LOG_MIN_FREE_BYTES = 100L * 1024 * 1024;
        public static final int INTERNAL_LOG_KEEP_NEWEST = 5;

        // Live stream for AdvantageScope's "Connect to Robot" (RLOG). 5800 is
        // taken by the Elastic layout WebServer; both are inside the
        // field-legal 5800-5810 range. VERIFY AdvantageScope's RLOG port
        // setting matches.
        public static final int RLOG_PORT = 5810;

        // --- Mechanism geometry for the visualizations (meters, from the CAD) ---
        // Arm length for both visualizations: pivot axis (through-bore
        // centerline) to the far intake roller axes, 13.9 in in the CAD
        // (9143-2025-A-0000 Leviathan STEP).
        public static final double ARM_LENGTH = 0.352;
        // 3D component poses (robot frame: X forward, Y left, Z up, origin at
        // the robot center on the floor), from the CAD with the carriage on
        // its hard stop and the floor at the wheel contact: the pivot axis is
        // 12.01 in forward of the frame center and 13.875 in above the floor.
        // VERIFY against an exported glTF model when one is attached: the
        // offsets must match its component origins (AdvantageScope docs:
        // "Custom Assets > Articulated components").
        public static final double ELEVATOR_X_OFFSET = 0.305;  // Meters forward of robot center
        public static final double ARM_PIVOT_HEIGHT = 0.352;   // Pivot height above the floor at the elevator hard stop (meters)

        // Indices into the component Pose3d array (RobotState/ComponentPoses)
        // published for AdvantageScope's articulated 3D robot model. This
        // robot's three moving components, in the order the code publishes
        // them (must match the component order in
        // advantageScopeAssets/Robot_Leviathan/config.json / the glTF export):
        public static final int MIDDLE_STAGE_INDEX = 0; // Elevator middle stage (rises at half carriage speed)
        public static final int CARRIAGE_INDEX = 1;     // Elevator carriage
        public static final int ARM_INDEX = 2;          // CorAl arm
    }
}
