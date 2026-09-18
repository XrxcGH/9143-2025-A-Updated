# Team 9143 - 2025 Robot Code (2026 Libraries)

This repository contains the code for Team 9143's 2025 FRC (Reefscape) robot, "Leviathan", running on the 2026 WPILib and vendor libraries. The drivetrain and the CorAl mechanism run Kraken X60s through CTRE Phoenix 6; the elevator runs NEOs on Spark MAX controllers through REVLib.

Everything described here has been run on the robot (September 2026) unless the text says otherwise. The items that are still unmeasured or untuned are collected in the [Pre-Competition Checklist](#pre-competition-checklist).

---

## Robot Overview

- **Swerve Drivetrain**: CTRE Phoenix 6 swerve on a 30"×30" frame — mixed modules (front: SDS MK4i, back: SDS MK4n), **all with L3+ drive gearing**, all Kraken X60 drive/steer, CANcoders, and a Pigeon 2. Drive/coupling ratios are uniform; only the steer ratios differ and are set per module.
- **Elevator Subsystem**: Dual NEO/Spark MAX elevator (15:1 MAXPlanetary) with MAXMotion height control in inches.
- **CorAl (Coral and Algae) Subsystem**: Pivoting arm + intake rollers with absolute-encoder-referenced angle control and CANrange game piece detection.
- **Superstructure**: Coordinates the elevator and the CorAl arm through a CAD-derived clearance map, so every preset is safe from any starting pose.
- **Vision System**: Three Limelights ("funnel", "barge", "reef") fusing MegaTag pose estimates into odometry, plus robot-frame AprilTag alignment.
- **LED System** *(code commented out — there is no CANdle on the robot)*: CTRE CANdle robot-state patterns, kept in [LEDs.java](src/main/java/frc/robot/subsystems/LEDs.java) for when one is installed.
- **Autonomous**: PathPlanner routines **and** Choreo trajectories, both selected from one dashboard chooser.

---

## Controls

One coral cycle is **LT → a face button → RT** on the operator's controller: index fingers and the right thumb. A pose button comes first; **RT always releases the piece**, coral or algae. Anything that drives the robot is a *hold*, never a toggle; the operator's manual sticks do nothing unless LB is held; anything rare or dangerous is disabled-only behind a 1 s hold, or Test-mode only. [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) is the single place where every binding is made.

### Driver (Xbox controller, port 0)
| Input | Action |
|---|---|
| Left stick | Field-centric translation |
| Right stick X | Rotation |
| **Left / Right trigger (hold)** | **Align** on an AprilTag while held: the LEFT / RIGHT reef branch, or centered on the coral station, barge or processor. Release = sticks back instantly |
| **Right bumper** | **SCORE** — the same gated command as the operator's RT; while aligning it also waits for *aligned* |
| Left bumper | **Driver heading zero**: the way the robot faces now becomes "forward" on the stick |
| A (hold) | X-lock the wheels (brake) |
| D-pad | Slow robot-centric nudges, all 8 directions |
| B, Back/Start + X/Y | Point modules / SysId — **Test mode only** |

- **Speed scaling.** Stick speed is scaled by the *Drive – Teleop Speed Scale* tunable (0.25 by default, for indoor testing) **and automatically by carriage height**: 100 % below 16.5", falling linearly to 40 % at 45" and above.
- **What the align triggers aim at** follows what the robot is doing. With a coral level selected, or with a coral in the claw and no level pressed yet: the LEFT / RIGHT **reef branch**. Stowed and empty: **centered on the coral station** (approached backward). Carrying an algae or at the barge pose: **centered on the barge or processor tag**, whichever is in view. For the centered targets either trigger does the same thing.
- **Hold a trigger + RB** fires the instant the robot is aligned **and** the scoring pose is reached.
- **Driver heading zero** moves only the driver's frame, never the pose estimator's heading, so it is safe at any time (point the robot away from you first). While disabled with no tag supplying a heading it also seeds the pose heading to alliance-forward; **Back + left bumper** forces that seed at any time.
- **SysId** applies open-loop voltage steps to the drivetrain, so those bindings (and B, point modules) exist only when Test mode is selected on the Driver Station.

### Operator (Xbox controller, port 1)
All position buttons run coordinated elevator + arm sequences through the **Superstructure**, which automatically avoids mechanism contact from any starting pose.

| Input | Action |
|---|---|
| **A / X / B / Y** | Coral **L1 / L2 / L3 / L4** (bottom / left / right / top of the face-button diamond) |
| **Right trigger** | **SCORE**: waits until the *measured* pose is reached, releases the piece, then **homes by itself** |
| **Left trigger** | **HOME**: stow and run the intake until a coral is detected; with an algae held, carry it low at the travel angle |
| **Left bumper (hold)** | **Manual take-over**: left stick Y = elevator, right stick Y = pivot, RT / LT = rollers out / in. **No collision interlocks** |
| Right bumper | Barge pose (holding pressure kept on); RT then fires the algae |
| D-pad up / down | Algae **high** / **low** intake — diagonals count |
| D-pad left | Algae hold in place: holding pressure on the rollers, arm to the safe travel angle |
| D-pad right | Raise the arm to the safe travel angle, in place |
| Back / Start, **held 1 s, disabled only** | Zero the elevator / the CorAl pivot (mechanism at its base) |

**Preset poses** (carriage height / arm angle):

| Pose | Height | Angle |
|---|---|---|
| Stow / intake | base (1.0", the hard stop) | 0° |
| Coral L1 | base | 100° |
| Coral L2 | 12" | 12.5° |
| Coral L3 | 30.5" | 25° |
| Coral L4 | 51.5" | 20° |
| Algae low / high intake | 20.5" / 37.5" | 160° |
| Algae hold, raise arm | unchanged | 100° (RAISE, the safe travel angle) |
| Algae carry (HOME with an algae) | base | 100° |
| Barge | 52" | 105° |

**Score (RT), in detail.**

- The gate is the *measured* pose: within **0.75"** and **3°** of the preset, with both mechanisms stopped. Pulled early, the trigger simply waits and fires on the loop the pose is reached, so it can never interrupt a move.
- Chain slack can leave the arm resting a few degrees off its target. Once both mechanisms have been still for 0.6 s, an arm within 8° counts as *landed*, so the score button can never be locked out. The height gate is never relaxed.
- Coral: the rollers run for at least 0.5 s and until the CANrange says the coral has left (1.5 s at most). If the coral is still detected, the pose is kept for a second try.
- **L2–L4 push the coral on through the claw (the intake's direction); L1 runs the rollers the other way** (`CORAL_L1_SCORE_SPEED`, −0.3, a starting value), so the coral leaves the way it came in. The autos' `ejectCoral` follows the same rule.
- Algae: rollers out for 0.5 s.
- Then the mechanism goes home by itself. After **L3 / L4** it does so only once the drivetrain has moved **0.35 m** from where it ejected, because that exit swings the claw 9–12" past the front bumper.

**Manual take-over (LB), in detail.** Pressing LB cancels the running move and brings both mechanisms to rest at their stopping distance. While it is held the left stick drives the elevator (gravity-compensated, so up and down feel the same) and the right stick drives the pivot (forward = claw forward); presets are locked out. Releasing LB holds position. The manual sticks bypass every collision interlock — watch the mechanism.

**Algae hold (D-pad left)** commands only the arm and the rollers. The carriage keeps the setpoint it already has: re-commanding the elevator to its own measured height would restart its motion profile for no reason.

**Rumble.** Coral acquired — both pads, one long buzz (leave the station). Pose reached — operator, two short (RT is live). Aligned **and** pose reached — driver, steady (fire). Scored at L3/L4 and waiting for room, or a move waiting for the reef to be clear — driver, slow pulse (back away). RT with nothing to score from — operator, one tick.

---

## Subsystems

### Swerve ([Swerve.java](src/main/java/frc/robot/subsystems/Swerve.java))
Extends the Phoenix 6 `SwerveDrivetrain` (high-frequency odometry and module control run in CTRE's code). Adds PathPlanner `AutoBuilder` configuration, vision pose fusion (with the required FPGA-to-Phoenix timestamp conversion), the AprilTag alignment command, SysId routines, and alliance-aware operator perspective.

**Mixed modules**: the front pair are SDS MK4i and the back pair SDS MK4n, both with **L3+ drive gearing** (5.36:1, 16T pinion) — so drive and coupling ratios are identical on all four modules, and only the **steer ratios differ** (MK4i 150/7:1 front, MK4n 18.75:1 back), set per module in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java). All driving (teleop and path following) uses **closed-loop velocity** so wheel speeds track the request regardless of battery sag. Top speed is ~5.96 m/s at 12 V, matched by PathPlanner's `maxDriveSpeed`.

### Elevator ([Elevator.java](src/main/java/frc/robot/subsystems/Elevator.java))
Two NEOs on Spark MAX controllers, each through a 15:1 MAXPlanetary reduction (5:1 × 3:1 cartridges), a 1:1 90° gearbox, and a ½" hex shaft driving 22T #25 sprockets — 5.5" of chain per sprocket rotation, doubled by the cascade rigging. The gearing gives **0.733" of carriage travel per motor rotation**, and encoder conversion factors scale everything to **inches**.

- **Height frame.** Heights are measured from the **top of the base-stage 2×1 to the bottom of the carriage 2×1**. In that frame the carriage sits at **1.000"** when it rests on its hard stop — the gap is the middle-stage tube, a 2×1 on its 1" side (*Elevator – Height At Hard Stop*). The encoder is referenced *to* that value rather than zeroed, so a commanded 20" lands the tape at 20". Travel is 1.0–53"; see [Calibrating the elevator height](#calibrating-the-elevator-height).
- **Follower.** The right controller is a hardware follower of the left and carries **no soft limits of its own**. REV documents only that a follower mirrors the leader's output, and since the follower spins opposite the leader its encoder counts negative as the carriage rises — leader limits copied onto it would keep its reverse limit in force for the whole climb and, if enforced, turn it into a brake the leader has to drag (slow, stuttering, current-limited climbs).
- **Control.** Height moves use MAXMotion profiles on the controller with on-controller kS/kV/kA/kG feedforward (REVLib 2026), so the carriage tracks smoothly and holds height at targets and when the operator releases the stick. Soft limits bound travel in every control mode, voltage compensation keeps response consistent as the battery sags, and configuration is persisted to flash so a brownout cannot revert it.
- **Setpoints are sent once.** MAXMotion regenerates its profile from the *measured* state every time a setpoint arrives, so `Elevator.setPosition` never re-sends the setpoint it is already running. `Elevator/Setpoint Count` on the dashboard must not move while the carriage is just holding.

**Gearing and tuning (15:1).** The NEO free speed through 15:1 is ≈ 69 in/s at the carriage; cruise is 40 in/s (≈ 6.9 V of kV) with 200 in/s², so full travel takes ≈ 1.5 s and a climb peaks near 9 V, leaving ≈ 3 V for the position loop on a sagging battery. Acceleration is limited by how hard the mast rings when the trapezoidal profile stops, not by the motors: 300 in/s² shook it visibly, and 200 costs only ≈ 0.07 s on a full-travel move.

Why the gains are what they are:

- **kA 0.0040 V per in/s²** carries the acceleration ramps. The carriage plus arm plus the reflected rotor inertia is ≈ 23 kg effective, so the ramps need about a volt. Without kA that force has to be bought with position error — about an inch of lag going up and an inch of lead coming down — which carries the carriage past every target and makes MAXMotion regenerate its profile around it (hunting).
- **kP 0.4 duty/in.** At 15:1 one inch of carriage error is only 1.4 motor rotations, so a given duty-per-inch is 3× weaker at the carriage than it would be at 45:1. With kD 0, the loop's only damping is the NEO's back-EMF: 0.4 gives a damping ratio of ≈ 0.63, where 0.5 (≈ 0.56) rings for two or three visible cycles at ≈ 6 Hz after any disturbance at a setpoint. If it still wobbles, 0.3 is ≈ 0.72. kD is not an option: the hall encoder resolves 0.0175", and a derivative of that at 1 kHz is noise.
- **kS 0, on purpose.** The Spark MAX applies kS as **+kS whenever the profile velocity is zero** — it does not follow the sign of the error — so a non-zero kS steps the feedforward by 2 × kS at the instant every *descent* ends (kG − kS moving down, kG + kS at rest), straight into that lightly damped loop. Moving friction is ≈ 0.2 V, which the position loop covers with ≈ 0.04" of lag.
- **Profile-error window 1.0".** This is how far the carriage may stray from the profile before the controller regenerates it from the measured state; it is not a settling tolerance. A sagging bus pushes the error past 0.5" near cruise, and a regenerated profile (built on a lagged velocity) is what then hunts. Every safety gate reads the measured height, so the wider window costs nothing.
- **Velocity filter 16 ms × 2** (≈ 24 ms of lag) instead of the Spark MAX default 32 ms × 8 (≈ 130 ms), because MAXMotion restarts its profile from the measured velocity.
- These values were chosen against a 1 kHz simulation of the loop and the CAD mass, across the plant uncertainty (mass 10–15 kg, friction, a held game piece, a sagging battery): worst case 0.24" of overshoot and no oscillation. On the robot the motion is smooth and continuous.

**kG — still to be measured.** Nothing counterbalances this elevator (the constant-force springs in the CAD are not on the robot), so the motors hold the whole moving mass: ≈ 145 N at the carriage by the CAD estimate, ≈ 9 A per motor. **kG ships at 1.0 V** (the mass estimate spans 0.8–1.3 V). Set it from **`Elevator/kG From Cruise`**: the mean of the applied volts at steady cruise going up and going down, which does not depend on friction or on the gains already configured. Run one long move each way from the Testing tab with the arm at RAISE (100°). `Elevator/Hold Volts` (applied volts at rest, with kS taken off) is a cross-check only — inside the static-friction band it mostly echoes the gains already set.

One real cost of the light gearing: with the robot disabled at height, brake mode lets the carriage creep down ≈ 3 in/s — stow before disabling.

### CorAl ([CorAl.java](src/main/java/frc/robot/subsystems/CorAl.java))
Pivot arm + intake rollers (both Kraken X60). Motion Magic in degrees, closed-loop holding at targets.

- **Angle reference.** The REV Through Bore absolute encoder is the angle reference: the motor's sensor is seeded from it at startup, at the start of a move **from rest**, and whenever the arm is idle — never mid-move, because a seed taken at 300°/s is ≈ 3° stale and would step the closed loop's feedback.
- **The zero survives a restart.** The raw through-bore reading at the zero position is stored on the roboRIO whenever the pivot is zeroed and **restored at the next code start**, so a restart with the arm raised (a brownout, a crash, a redeploy) does not take "wherever the arm is" as 0° and shift every gate and both soft limits. A restored angle outside the arm's travel is refused with a dashboard alert (the encoder has moved on its shaft: re-zero at the base).
- **Landing correction.** The loop closes on the rotor and the chain has slack, so at the end of a sweep the arm's inertia and gravity carry it through the slack: the real arm rests a few degrees *past* the target with the rotor exactly on it. Once the arm has been still for 0.2 s, if the through bore is more than 0.75° off, the rotor is re-seeded from the through bore and the same target re-issued, so the loop drives out the difference — at most three times per target, so it cannot hunt inside the slack.
- **Gravity feedforward** works although 0° is not horizontal (`GravityArmPositionOffset`, from the *Pivot – Balance Angle* tunable). ***Pivot – kG* ships at 0 until measured**, and the 33° balance angle is an estimate — see the tunables table for the procedure. Without kG the arm rests 2–3° low at RAISE and the algae poses.

**Game piece detection is by signal strength, not distance.** Measured on this claw: a held coral returns 0.04–0.05 m at a **saturated 65535** signal strength, while an empty claw returns *about the same distance* at **3000–4000**. That is the first of the two setups in CTRE's CANrange tuning guide — the sensor looks into open air, so the distance reading means nothing when nothing is held, and the strength is what separates the two, by a factor of sixteen. So the strength gate does the work (*CorAl – Coral Min Signal Strength*, **15000**: roughly four times the empty return and far below a held one) and the distance threshold is deliberately generous (**0.30 m**, well above the 0.05 m a coral reads), as the guide prescribes. CTRE's default strength gate of 2500 is below the empty claw's return and would report a coral that is not there.

The verdict is formed in code rather than from the sensor's own proximity bit, which can only mean "closer than the threshold". It latches on both sides: a return that has counted as a coral keeps counting until the strength falls to 75 % of the gate, a distance reading inside the hysteresis band keeps the previous verdict, readings the sensor flags as compromised are rejected, and both edges are debounced for 0.3 s. A confirmed arrival stops the intake automatically.

**If it ever misreads**, read `CorAl/CANrange Distance` and `CorAl/CANrange Signal Strength` empty and holding a coral, and set whichever one separates them: *CorAl – Coral Min Signal Strength* if the strengths differ, *CorAl – Coral Detect Distance* (with *Coral Detect When Closer (1) Or Farther (0)*) if the distances do. *CorAl – Coral Detection Enabled (1) Or Off (0)* set to 0 runs the intake on the operator's button alone in the meantime, so a confused sensor never blocks the robot.

### Superstructure ([Superstructure.java](src/main/java/frc/robot/Superstructure.java))
The command factory for all coordinated elevator + CorAl motion. The free region of (elevator height, arm angle) was computed from the robot CAD — every CorAl part's true outline swept about the pivot against the base stage, the middle stage (which rises at *half* the carriage travel), the funnel, frame and bumpers — and validated against the two contacts measured on the robot (0° at ≈10.75", 5° at ≈23", both reproduced within ½"). See [CAD clearance analysis](#cad-clearance-analysis-claw-vs-elevator) for the map.

**How a move runs.** Every move is planned from the mechanisms' **measured** state when the button is pressed, and runs as staged steps whose handovers are measured heights and angles, never timers — so the sequences are safe at any elevator or pivot speed, and only their duration changes. Buttons are safe in any order at any time, including pressing a new one mid-move. What the free region forces:

- **Low box.** Any arm angle from 8° up is clear below 16.5", so base→L2 and base→L1 move both mechanisms together. Tucked (< 8°) is allowed only below 8.5" (the cross bar under the top sprockets).
- **Climbing out** needs the arm at ≥ 75° — the middle-stage top tube blocks roughly 18–30" for lower angles — and from **RAISE = 100°** the carriage may go anywhere. **90° is not a travel angle**: the claw's lower rear meets the middle-stage top tube from ≈ 36" up. Ascents give the carriage a ratcheting head start (8.5" while tucked → 16.5" once clear → 36" past 75° → anything once the arm reads RAISE).
- **L3 (30.5", 25°).** The carriage climbs straight to the target at RAISE. The arm leaves RAISE as the carriage passes **24"** and sweeps down during the last six inches of the climb, held row by row by the **arm-side clamp** (below) until the carriage has opened each one; the last row (25–30°) opens just above 30", as the carriage arrives. Leaving, the carriage lifts to 31", clear of the tube, and then heads for the **real target** — up or down — as the arm passes 75°; descending any earlier would sweep the claw into the middle-stage top tube.
- **L4 (51.5", 20°).** The top of travel is clear only at ≤ 22.5°, and while the arm is between 95° and 70° the carriage may not be above 36–39". So the carriage climbs to a **35.5" clamp** with the arm parked at RAISE, the arm is released as the carriage passes **24"**, and from there the two run **together**: the clamp opens as the arm comes down (36" at 95°, 40" at 70°, 48" at 40°) up to the **48" pre-top height**. Below 37" the arm is held at the **27.5° stage angle**, the middle of the 25–30° row.
- **The L4 pre-top gate.** The last three and a half inches wait for the arm to **actually reach 22.5°** — compared strictly, not with the 3° tolerance the travel gates use. This gate is not negotiable: the corridor table says 25° is clear to 51", and on the robot it is not — a carriage climbing past 48" with the arm still at 25° puts the claw into the top bar.
- **Leaving L4** mirrors the climb: down to 39" at the L4 angle, the arm released at 43" and heading for RAISE in one sweep under the arm-side clamp, the carriage to the **33" station** once the arm is past 40°, and on to the real target once the arm clears 75°.
- **Why L4 is 51.5" and not higher.** Half an inch under the 53" forward soft limit leaves the carriage fighting the top of travel, and 52.5" sits exactly on the 20° corridor ceiling. Raise it toward 52" if the coral needs the height, not past it.
- **Algae poses (160°)** hit the bumper below ≈ 7.5", so the arm waits at 110° until the carriage is above 8" — and, leaving an algae pose for anything low, the arm comes back to RAISE before the carriage drops below 8".
- **Stow** tucks to 0° once the descending carriage passes 8.5".

**Why the arm-release height is 24" and not lower.** 75–100° is clear from the base to 36" by the table, so the release height is not a table limit. Below ≈ 22" the claw's tail bar passes the funnel's sheet-metal lips, which the CAD clearance model **does not include**; at RAISE it is well behind them, and at 24" it is ≈ 2.5" above them before the arm moves. **Do not lower `ARM_RELEASE_MIN_HEIGHT` without a jog check**: jog the arm to 75° and raise the carriage slowly from 12" to 24" with eyes on that gap.

**Nothing waits.** The invariant is that the elevator, the CorAl, or both are always under way until the pose is reached — there is no phase where the mechanisms sit still. The gates between phases are conditions on *measured* state that release the next command; they never stop what is already moving. Where the corridors force one mechanism to hold (the carriage cannot climb past 48" until the arm is inside 22.5°), the other one is doing the work that opens the gate. Leaving a pose, the carriage travels toward the target — up *or* down — while the arm swings to RAISE; coming home, the arm starts round as soon as the carriage is inside the low box and finishes the tuck when it drops below 8.5".

**L3 ↔ L4 goes straight across.** Both poses live in the upper corridor and their bands overlap over **36–51"**, so the arm turns only the five degrees between 25° and 20°, at the near edge of that overlap: climbing, it turns at **36"** and keeps climbing; descending, it turns at **48"** and keeps dropping. Going round by RAISE would be 150° of rotation to avoid five. The carriage is never *sent* to the turn height — it heads for the real target from the first loop, and the turn height is only the threshold the arm waits for as the carriage goes past, because a carriage commanded to a height and resting 0.02" short of it would never open a gate that waits for that exact height. Simulated with the profiles the robot runs, the transfer takes ≈ 0.7 s each way.

Checked against every ordered pair of operator poses, L3 ↔ L4 is the only pair with such a shortcut, and that is geometry rather than luck. Nothing crosses between the low box and the upper corridor, because the middle-stage top tube sits between them at every scoring angle — the arm *has* to come up to 75–100° to cross, so for those moves the excursion **is** the move. Everything else (L1, the algae poses) is already at or past a travel angle, so its plan has no excursion to remove. The turn never happens above the 48" pre-top height, for the same reason that gate exists at all.

**Two clamps, one table.** Each mechanism is limited every loop by the other's **measured** position, by a walk of the same 32-row table:

- *Carriage by arm* (`ceilingForSweep` / `floorForSweep`): the carriage is given its **final** height, clamped to what the corridors allow for the angles the arm still has to sweep; as the arm advances the clamp opens. A legal height is commanded as it is, an illegal one half an inch short of the limit. The walks **fail closed**: a pose outside the table, or a carriage that has overshot a row the arm is about to enter, holds.
- *Arm by carriage* (`armLimitForHeight`): the arm is given its **final** angle, clamped 2.5° short of the first row the carriage's height has not opened (a row counts as open 0.1" inside its band). Without this half the arm could only be *gated* — parked at RAISE until the carriage reached a window where its whole sweep was clear at once — and the carriage would arrive at its clamp with the arm barely moving, and stop there.

Because each clamp reads the other mechanism's measured state, the pair stays inside the corridors at any relative speed, including one of them stalling, and the margins are nested (0.1" inside ½") so neither can wait on the other. `SuperstructureClampSimTest` steps both axes through the L3 climb, the L4 climb and the L4 exit — nominal, arm at half speed, carriage at 60 %, carriage stalled for a second mid-move — and fails if the pose ever leaves the table, the pre-top rule is broken, or the move does not finish.

**Paced to travel together.** The arm is the long pole of every move into or out of an upper pose; at full pace the carriage reaches each clearance limit before the arm has opened it, brakes, and is released again — sequential and jerky even when nothing ever stops. So each move runs the carriage at a **pace** (`Pace`: FULL / BRISK 0.8 / EASY 0.6 / SLOW 0.4 of the tuned cruise *and* acceleration — one Spark MAX closed-loop slot each, so the pace is simply part of the setpoint) chosen so that it never has to brake for the arm. The arm is also released **0.2 s before** the carriage reaches the 24" release height (held at 92.5° until it is actually there), which is the difference between braking for the 35.5" clamp and sailing through it. The choices come from `SuperstructureSequenceSimTest.pacesCompared` (`build/sim/paces.csv`: time, hitches and slowest mid-move speed for every pace), and the sim asserts that the scoring climbs have **zero** hitches:

| Move | Pace | Sim time | Carriage |
|---|---|---|---|
| BASE / L1 / L2 → L3 | FULL | 1.5–1.6 s | one run, never below cruise |
| BASE / L1 / L2 → L4 | BRISK | 1.8–2.0 s | one run at 32 in/s (FULL is 1.66 s but only *just* hitch-free; a real arm lags its profile) |
| L3 ↔ L4 | FULL | 0.7 s | one run |
| L4 → BASE / L2 | FULL to 41.5", then SLOW to the station, FULL below | 2.0 s | never below ≈ 7 in/s (FULL throughout takes the same 2.0 s with two dead stops) |
| algae pose → L4 | EASY | 1.5–1.9 s | the arm has 140° to come round; the carriage still waits at 35.5" |

A carriage **at rest** also does not set off on a short run (< 8") to a clamp, which it would only have to brake for; it waits the ≈ 0.2 s until the arm has opened a proper run.

**Setpoints are latched.** MAXMotion regenerates its profile from the *measured* position and a lagged velocity every time a setpoint arrives, so a clamp re-sent every 20 ms restarts the profile fifty times a second — which shows up as hunting. A new height is therefore sent only when the clamp really moves (small advances are held back while the carriage is not yet braking for the old one), and `CorAl.setPivotAngle` ignores a repeat of its target. Arm-arrived waits end on at-target **or stopped**, so chain slack leaving the through bore a degree outside its window cannot park both mechanisms until a timeout. A second button press mid-move re-latches the arm as well as the carriage, and the roller commands (`ejectCoral`, `rollersRaw`) require no subsystem, so they never cancel a move.

Per-loop geometry is deliberately a **table walk, not a search**: a route search over a grid of the corridor map costs millions of lookups inside a command's initialization, which overruns the robot loop.

**Held on enable.** Disabling cuts both mechanisms' outputs; on the next enable `RobotContainer.enabledInit()` latches a closed-loop hold at wherever they are, so a carriage that was up (the scoring autos end at L4) does not sink with the arm un-held until the operator presses something.

**Reef safety.** The claw reaches **9–12" past the front bumper** at 75–100° (CAD), at the height of the reef's branches, and every move into or out of L3 / L4 swings it through there. Three things follow:

1. Such a move **waits** while a reef tag is right in front of the bumper (< 0.80 m ahead; `Superstructure/Near Reef`) and starts by itself once the robot is clear, with the driver's pad pulsing. L3 ↔ L4, the low poses, the algae poses and the manual take-over are never held.
2. The L3 / L4 **alignment is two-stage**: it lines up on the branch 0.40 m back from flush, waits there for the scoring pose, then closes to flush, and backs out to the standoff after the score (which is also what releases the automatic stow).
3. The **autos** do not raise, eject and stow *in place* at the reef: `parallel[path, prepL4] → ejectCoral → parallel[path, stowAfterBackingOff]` has the arm through its sweep and behind the bumper with the robot still ≈ 0.7 m out, and is ≈ 3.5 s faster per scoring leg than doing all three in place.

**Tested as the robot runs it.** `SuperstructureSequenceSimTest` schedules the *real* commands on the real `CommandScheduler` with a stepped clock against a simulated carriage and arm. (The Superstructure talks to the `CarriageAxis` / `ArmAxis` interfaces, which `Elevator` and `CorAl` implement; the vendor simulators run on wall-clock time and cannot be stepped.) It covers all 56 pose-to-pose moves, the same with chain slack, a half-speed arm and a 60 % carriage, 245 "second button mid-move" interrupts, the score → back off → home cycle, the reef hold, the algae hold and the L1 eject direction. Every 5 ms it requires the pose to be inside the clearance table and the 48" rule to hold; per move, that it finishes at its pose, the carriage's setpoints do not alternate, neither controller is flooded, and the mechanisms are never both stationary. Setpoint lists and timings for every move land in `build/sim/*.csv`.

`SuperstructureCorridorTest` pins the geometry and the overlaps: the corridor table's shape, the measured contacts, every preset pose, the staged waypoints, the fail-closed walks, lookups from a resting pose that reads a hair under its own row (L3 rests at 24.x°, which is the 20–25° row, so lookups nudge the measured angle onto the row that holds the height), and — for each sweep — the carriage target the robot would command at every arm angle, which must keep both the pose and the path to it inside the corridors and must actually arrive.

### Vision ([Vision.java](src/main/java/frc/robot/subsystems/Vision.java))
Sends the estimated heading to each Limelight every loop and fuses the returned **MegaTag2** poses into the drivetrain with distance/tag-count-scaled confidence. While **disabled** it fuses **MegaTag1** instead, whose solve carries an absolute heading, so the pose heading is field-correct before the match starts. Requires Limelight OS 2026.0+.

- Each camera frame is fused **once**: the NT sample timestamp identifies a frame, and re-adding the same sample every loop would collapse the estimate onto the raw camera pose.
- The cameras' estimates are inserted oldest-first (inserting an older measurement discards newer corrections).
- Estimates are rejected when off-field, when a MegaTag2 solve averages more than 6 m to its tags, when a single-tag MegaTag1 solve is ambiguous (> 0.7) or far (> 3 m), and while the robot is spinning fast.
- Each camera's last fused pose is drawn on the Field widget.

**Alignment** works in the **robot frame**: each camera's primary tag is converted from Limelight camera space into "where is the tag relative to the robot center" using that camera's mounting pose (lens offset, yaw, pitch — `Vision.tagPositionInRobotFrame`, pinned by `VisionGeometryTest`), so the yawed reef camera and the rear funnel camera drive the same loop with no per-camera mirroring.

- Translation is **one P controller on the error vector**: speed from its length (with a 0.12 m/s floor), direction along it. The direction of travel therefore turns smoothly; two independent per-axis controllers snap it each time an axis crosses its own deadband, which whips the swerve modules round and shakes the robot.
- It stops when the forward error (3 cm) and lateral error (1.5 cm — a coral has only ~3 cm of clearance on a branch) are both inside their deadbands, and **stays stopped** until one passes 1.6× its deadband.
- Heading is a second P loop to the heading that is **square to the tag's face**.
- Everything is **slew-limited** (3 m/s², 6 rad/s², including the ramp-down when the tag is lost).
- "Told to move, not moving, nearly there" counts as **arrived by contact** rather than stalling against the reef.

**The servo never drives on a raw camera solve.** A tag does not move, so each *new* camera frame is turned into a **field position** for the latched tag — using the pose the robot had **when the image was captured** (`samplePoseAt`), not now — low-pass filtered there behind a 3-frame outlier gate, and every loop the tracker gets that point seen from the **current odometry pose**. Filtering a static point adds no control lag, and the 50–100 ms camera latency (which would otherwise appear as a phantom lateral error of *range × heading change* whenever the robot is turning), single-frame solve noise and dropped frames never reach the wheels.

**The square heading does not need a field-true gyro.** The tag's square heading comes from the 2025 AprilTag layout (the authored paths' end rotations agree with it, which the test checks), but the pose heading is in whatever frame the gyro was last zeroed — compared directly after a driver re-zero, the robot would square up to the wrong direction. So each frame's **MegaTag1** solve (a field-true heading from the tag geometry alone) is compared with the pose heading at capture, the difference is filtered (`Vision/Heading Offset`), and the square heading is converted into the pose estimator's own frame before the servo sees it.

**Vision never moves the driver's frame.** Field-centric driving steers relative to the pose heading, and MegaTag1 re-seeds that heading whenever the robot sits disabled looking at a tag. If the driver's "forward" followed it, aligning on a tag, disabling in front of it and re-enabling could leave the sticks pointing the wrong way. So the driver's forward is kept as a direction in the **raw gyro frame** (which only the gyro moves), and the operator perspective is recomputed every loop to cancel whatever vision or a pose reset did to the pose heading (`Swerve.periodic`). It changes only when the driver zeroes it (left bumper), when an auto resets the pose to the field's frame, when the alliance changes, or — **until the driver has zeroed it** — while disabled with a *converged two-or-more-tag* heading seed, so a robot that booted facing any which way and was placed on a real field still gets a correct "forward" without anyone touching it. One reef tag in front of the bumper is not a strong seed, and after a driver zero nothing from vision counts.

**Camera roles and the tag latch.** Cameras only supply their own tag class to the aligner: funnel → coral stations, reef → reef, barge → barge and processor (**nothing until its mounting pose is measured**). Every camera still feeds pose estimation with whatever it sees. The first tag chosen is **latched** (and sent to the cameras as the priority tag). While the latched tag is out of view its filtered position keeps being **carried on odometry** for up to 1.5 s, so the goal cannot flip between adjacent reef faces and an approach whose tag leaves the camera at the end still finishes. The rear funnel camera (pitched 50° up) loses the station tag about 0.55 m from the wall, before the bumpers are flush, so that last stretch is dead-reckoned (`Vision/Target From Memory`). `Vision/Forward Error`, `Vision/Lateral Error`, `Vision/Heading Error` and `Vision/Aligned` show the servo state.

**Tracking goals** (where the tag should sit relative to the robot center; 2025 Reefscape tags):

| Tags | Goal |
|---|---|
| Reef (6–11, 17–22), L2 selected | Tag `REEF_FLUSH_DISTANCE` ahead (bumpers flush) and `REEF_BRANCH_OFFSET` to the side: centered on the driver-selected **left/right branch**, square to the face |
| Reef, L3 / L4 selected | The same branch, held `REEF_STANDOFF_EXTRA` (0.40 m) back until the scoring pose is reached, then flush; back out to the standoff after the score |
| Reef, L1 selected | Tag **1 m** ahead, centered, square (room for the arm to swing to 100°) |
| Reef, algae intake | Flush and centered, square |
| Coral stations (1, 2, 12, 13), stowed and empty | Tag `STATION_FLUSH_DISTANCE` **behind** the robot (rear bumpers flush), centered, square — approached backward on the rear funnel camera |
| Barge (4, 5, 14, 15), carrying an algae or at the barge pose | Tag `BARGE_SCORE_DISTANCE` (1.2 m) ahead, centered, square |
| Processor (3, 16), carrying an algae or at the barge pose | Tag `PROCESSOR_DISTANCE` (0.55 m) ahead, centered, square |

- **Barge and processor alignment is not available yet.** The front "barge" Limelight is the only camera that can see those tags with the robot facing them, and it supplies nothing to the aligner until `VisionConstants.LIMELIGHT_POSES[1]` is measured and marked `measured = true`. `BARGE_SCORE_DISTANCE` and `PROCESSOR_DISTANCE` are first guesses, to be tuned once it is.
- The flush distances are geometry — half the bumper-to-bumper length (0.464 m) plus a little standoff, 0.47 m by default. The dashboard's `Vision/Distance` (forward, negative behind) and `Vision/Lateral` (positive left) report the same robot-frame numbers, so tune by pushing the robot into position and copying them.

**Camera mounting (MegaTag camera poses).** Each Limelight needs its lens position and orientation on the robot so MegaTag can turn "where the tag is in the image" into "where the robot is on the field". These live in `VisionConstants.LIMELIGHT_POSES` and are pushed to the cameras at startup — so they are version-controlled and survive a camera reset — but only for cameras marked *measured*. Limelight's robot-space convention: origin at the frame center on the floor, **X forward, Y toward the robot's right** (opposite of WPILib), Z up; pitch positive = lens tilted up; yaw = lens heading (180° = rear-facing).

| Camera | Forward | Side | Up | Pitch | Yaw | Status |
|---|---|---|---|---|---|---|
| funnel (rear) | −14.0" (1.0" inside the back edge of the frame) | 0 (centered) | 29.625" | +50° (up) | 180° (rear-facing) | **measured** |
| barge (front, high) | — | — | — | — | — | **placeholder**, `measured = false` |
| reef (front-left, base stage) | +11.04" | −11.25" (left) | 15.78" | −20° (down) | −30° (toward the centerline) | **from the CAD** (lens-barrel axis of the Limelight 3G model) |

The CAD has the barge camera's mount (front-left, ~40.6" up, 13.4" forward) but no camera body in it, so there is nothing to derive its pose from: measure it on the robot, fill in `LIMELIGHT_POSES[1]`, and set `measured = true`.

One-time check after deploying: open `http://limelight-funnel.local:5801` and confirm the 3D preview shows the camera at the back, pointing rearward and tilted up; `http://limelight-reef.local:5801` should show the camera front-left, low, looking forward-right and down. If either points the wrong way, flip the pitch or yaw sign in the constant. With the funnel camera tilted 50°, the *Station Flush Distance* is still simply whatever `Vision/Distance` reads when the rear bumpers are flush — the tilt is accounted for in the robot-frame conversion.

**Heat and fan noise.** The code does two things to keep the Limelights cool without giving up tracking performance. The **LEDs are never turned on** (AprilTags need no illumination, and the LED array is the camera's biggest heat source). Processing is **throttled while the robot is disabled** (one frame processed per 100 skipped — still ~1 solve/s for the pre-match heading seed), with full rate restored the instant it enables. **Selecting Test mode on the Driver Station lifts the throttle without enabling**, so the dashboard's vision readouts are live when checking a camera on the bench; throttled, they can be a few seconds behind the camera's own stream. The rest is configured on each camera's web UI (`http://limelight-<name>.local:5801`), where these settings dominate CPU/GPU load and therefore fan speed:

| Setting | Recommendation |
|---|---|
| Pipeline type | **AprilTag** only — make sure no retroreflective/neural pipeline is selected by default |
| Detector downscale | **2** (or 3): halves the detection workload; corner refinement still runs at full resolution, so accuracy is essentially unchanged at reef distances |
| Capture resolution / FPS | The lowest mode that still detects tags across the field — 640×480 @ 90 fps is plenty |
| Stream resolution | Lowest offered; the MJPEG encoder for the dashboard feed is pure overhead |
| Multi-target / 3D solve | Leave **enabled** — MegaTag2 depends on it and it is cheap relative to detection |
| Camera exposure/gain | Shorter exposure lowers sensor heat slightly; keep just enough to detect tags reliably |

If a camera still runs hot with those, check its mounting: the fan intake needs clear airflow, and a camera boxed in next to a motor or the roboRIO will run warm regardless of settings.

### LEDs ([LEDs.java](src/main/java/frc/robot/subsystems/LEDs.java)) — **commented out**
There is no CANdle on the robot, so the whole subsystem is commented out to keep the device off the CAN bus and the class out of the build: every line of `LEDs.java`, `LEDConstants` in `Constants.java`, and the lines marked "CANdle disabled" in `RobotContainer.java` and `Dashboard.java`. The dashboard's *LEDs/State* field reads "not installed". Restore all of them together when a CANdle is installed. What the code does when enabled — state derived automatically each loop, no commands needed:

| Priority | State | Pattern |
|---|---|---|
| 1 | Disabled | Alliance-colored Larson scanner |
| 2 | Autonomous | Rainbow |
| 3 | Game piece held | Solid green |
| 4 | Vision tracking | Strobing cyan |
| 5 | Endgame (last 20 s) | Strobing yellow |
| 6 | Teleop idle | Solid alliance color |

Set `LEDConstants.LED_COUNT` to match the installed strip (8 onboard LEDs + strip length) and adjust `StripType` in LEDs.java if colors look swapped.

### Telemetry ([Telemetry.java](src/main/java/frc/robot/Telemetry.java))
Publishes drivetrain state to NetworkTables (for AdvantageScope/Elastic) and CTRE SignalLogger (.hoot logs) at the odometry rate, plus Mechanism2d module visualizations.

---

## Dashboard (Elastic)

All dashboard integration goes through **[Elastic](https://frc-elastic.gitbook.io/docs)** and is centralized in [Dashboard.java](src/main/java/frc/robot/Dashboard.java) — subsystems expose getters and publish nothing themselves. Elastic has no Shuffleboard-style layout API, so this project publishes plain NetworkTables data (under `/SmartDashboard`) plus a layout file, [elastic-layout.json](src/main/deploy/elastic-layout.json).

### Tabs
| Tab | Purpose | Highlights |
|---|---|---|
| **Setup** | Pre/post-match checks | FMS info, battery + CAN health, Alerts, Zero Elevator / Zero Pivot buttons, sensor status, **all three Limelight feeds** with per-camera "sees a tag" lights and, under them, **Best Tag**, **Align Tag** and **Visible Tags**, auto chooser, branch side |
| **Autonomous** | Auto selection & monitoring | Auto chooser, big match timer, **Field widget with live robot pose**, game piece indicator, Alerts |
| **Teleop** | Driving | **Field widget**, match timer, big game-piece box, swerve module widget, vision tracking + **branch side**, elevator/pivot position bars and at-target lights |
| **Testing** | One-mechanism testing + diagnostics | **Elevator / Pivot / Intake setpoint sliders with Go/Run/Stop buttons**, **Tunables (Robot Preferences) editor**, command scheduler, subsystem widgets, current graphs, **Best Tag**, the alignment tag's distance and TX (*Align Dist*, *Align TX*), CANrange readout |

Every tab fits **12 × 5 grid cells** (1536 × 640 px at grid size 128), which leaves room for a docked Driver Station on the drive laptop. To change the layout, edit it in Elastic, `File → Export Layout`, and commit the exported file back to `src/main/deploy/elastic-layout.json`. The robot **switches Elastic to the right tab automatically** on mode changes (disabled → Setup, auto → Autonomous, teleop → Teleop, test → Testing) via ElasticLib ([util/Elastic.java](src/main/java/frc/robot/util/Elastic.java)). The Zero buttons on the Setup tab act only while the robot is disabled.

### Vision readouts
Two separate questions are answered on the dashboard, and they deliberately use different data.

**"What do the cameras see?"** — unfiltered, so it always agrees with the camera streams:

| Topic (under `/SmartDashboard`) | Meaning |
|---|---|
| `Vision/Best Tag` | ID of the **closest tag any camera reports**, −1 when none. No tag-class, camera-role or mounting-pose filter is applied |
| `Vision/Best Tag Camera` | Name of the camera that sees it (`funnel` / `barge` / `reef`) |
| `Vision/Visible Tags` | Every tag ID per camera, e.g. `funnel: 12 \| reef: 18 19` (Setup tab, under the camera streams) |
| `Vision/<camera> Has Target` | Per-camera "sees a tag" light |

**"What can the aligner use?"** — the closest tag a camera may **align** on (the tag is in that camera's class and the camera's lens pose is measured), ignoring the superstructure goal and the latch so the readouts work with the robot pushed into position while disabled:

| Topic | Meaning |
|---|---|
| `Vision/Alignment Tag` | ID of that tag, −1 when none |
| `Vision/Distance`, `Vision/Lateral` | Its robot-frame position: forward (negative = behind) and left, meters |
| `Vision/TX`, `Vision/Square Heading` | Its horizontal angle in the camera, and the field heading square to its face |

`Vision/Alignment Tag` = −1 while `Vision/Best Tag` shows an ID means the tag is seen but is not one that camera aligns on — for example anything seen only by the barge camera while its pose is unmeasured, or a reef tag seen by the funnel camera.

- A camera whose tag list has not changed for 10 s is ignored by these readouts, because NetworkTables keeps the last value of a camera that lost power or its link.
- While disabled the cameras are throttled to about one solve per second, so the readouts lag the streams. **Select Test mode on the Driver Station** (no need to enable) to lift the throttle for bench checks.
- *Best Tag* and *Align Tag* sit side by side on the Setup tab, so a tag that is seen but not aligned on shows at a glance. `Vision/Best Tag Camera` has no widget in the shipped layout (*Visible Tags* already names the camera); drag it in from Elastic's topic list if you want it.
- **After deploying this release, re-download the layout** (`File → Load Layout From Robot`) on every drive laptop — the *Best Tag* / *Align Tag* / *Visible Tags* widgets on the Setup tab appear only once the new `elastic-layout.json` is loaded.

### Testing one mechanism at a time
The Testing tab drives each mechanism **independently** — set the *Elevator Setpoint* slider and press *Elev Go*, and only the elevator moves; the arm stays exactly where it is. The same goes for *Pivot Setpoint / Pivot Go* and *Intake Speed / Run / Stop*.

These single-mechanism moves still consult the collision model in [Superstructure.java](src/main/java/frc/robot/Superstructure.java), but instead of moving the *other* mechanism out of the way (as the preset buttons do), an unsafe request is simply **refused with a toast notification** that says why — for example "Elevator test move refused: 30.0 in is not reachable with the arm at 0 deg (CAD contact band). Put the arm at 100 deg (Pivot Setpoint 100 + Pivot Go) for full travel." With the arm at 100° the elevator can be run through its full travel on its own; the refusals come from the CAD corridor table.

### Live tuning without redeploying (Tunables)
Empirically measured numbers live in [util/Tunables.java](src/main/java/frc/robot/util/Tunables.java), backed by **WPILib Preferences**: they appear in the Testing tab's *Tunables* widget, and the roboRIO **persists them to disk** — they survive reboots, power cycles, *and* code deploys. Every getter clamps its value to a sane range, so a typo on the dashboard cannot command something dangerous.

- **When an edit takes effect:** driving and vision values on the next loop; values the Superstructure reads at plan time on the next button press; elevator, pivot and CANrange controller values the next time the robot is disabled.
- **Defaults vs. stored values.** The values in `Constants.java` are only the factory defaults; *Reset Tunables to Defaults* restores them. Because stored values survive a deploy, **changing a default in `Constants.java` does nothing on a robot that already has the key stored**. Bump `DEFAULTS_VERSION` in `Tunables.java` (which overwrites every tunable once at the next boot) or press *Reset Tunables*. When only a few defaults move, add a targeted migration block in `Tunables.init()` instead, so everything else tuned on the dashboard survives.

| Tunable | Default | What it sets |
|---|---|---|
| Drive – Teleop Speed Scale (0–1) | 0.25 | Fraction of top speed *and* rotation rate at full stick. 0.25 is for indoor testing; raise toward 1.0 for competition |
| Elevator – Travel Ratio (measured / modeled) | 1.0 | Carriage inches per motor rotation = 0.733 × this. Confirmed 1.0 on the robot; it exists only for a re-check |
| Elevator – Height At Hard Stop (in) | 1.000 | Where the carriage sits on its hard stop, in the preset height frame. The encoder is referenced to it and the reverse soft limit sits here |
| Elevator – kP (duty per in) | 0.4 | Spark MAX position loop gain |
| Elevator – kS (V) | 0 | Static feedforward; deliberately zero (see [Elevator](#elevator-elevatorjava)) |
| Elevator – kG (V) | 1.0 | Gravity feedforward. **Set it from `Elevator/kG From Cruise`** |
| Elevator – kV Scale (× free-speed model) | 1.0 | Multiplies the NEO back-EMF velocity feedforward, which is derived from the travel ratio. Overshoot → lower it |
| Elevator – kA (V per in/s²) | 0.0040 | Acceleration feedforward on the profile. Lags on the ramps → raise; leads into the target → lower |
| Elevator – Cruise Velocity (in/s) / Max Acceleration (in/s²) | 40 / 200 | MAXMotion profile (full travel ≈ 1.5 s). The staged sequences are gated on measured state, so a change only alters their timing |
| Elevator – Profile Error (in) | 1.0 | How far the carriage may stray from the profile before it is regenerated from the measured state. Not a settling tolerance |
| Pivot – Cruise Velocity (deg/s) / Acceleration (deg/s²) / Jerk (deg/s³) | 300 / 800 / 6000 | CorAl Motion Magic profile |
| Pivot – kG (V, claw horizontal) / Balance Angle (deg) | 0 / 33 | Pivot gravity feedforward. **kG is 0 until measured** and the balance angle is an estimate |
| CorAl – Coral Min Signal Strength | 15000 | CANrange return strength above which a reading can count as a coral. This is what separates "holding" from "empty" on this claw |
| CorAl – Coral Detect Distance (m) / Hysteresis (m) | 0.30 / 0.015 | Distance threshold and the band on both sides of it; deliberately generous, since the strength gate does the work |
| CorAl – Coral Detect When Closer (1) Or Farther (0) | 1 | Which side of the distance threshold a held coral puts the reading on |
| CorAl – Coral Detection Enabled (1) Or Off (0) | 1 | Master switch: 0 runs the intake on the operator's button alone |
| Vision – Reef Flush Distance (m) | 0.47 | Robot center to the reef tag with the front bumpers flush (L2–L4 and algae) |
| Vision – Station Flush Distance (m) | 0.47 | Robot center back to the station tag with the rear bumpers flush |
| Vision – L1 Score Distance (m) | 1.0 | L1 standoff |
| Vision – Barge Score Distance (m) / Processor Distance (m) | 1.2 / 0.55 | **First guesses** — unusable until the barge camera's pose is measured |
| Vision – Reef Branch Offset (m) | 0.165 | Tag center to branch center |
| Vision – Tracking Distance kP / Rotation kP | 1.5 / 0.06 | Alignment servo gains (m/s per m, rad/s per degree), clamped to 0 – 3× the default so a mistyped value cannot invert or destabilise the servo |

Notes on the table:

- **Elevator calibration values** (travel ratio, hard-stop height) are applied only with the carriage at its hard stop while disabled, where the encoders re-reference; an info alert shows while an edit is waiting.
- **Pivot profile.** The pivot chain has slack, and Motion Magic decelerates at the rate it accelerates, so the *acceleration* sets how hard the chain catches the arm at the end of a move and the *jerk* sets how abruptly the stop begins (6000°/s³ is a 0.13 s ramp). The scoring rotations are about 75°, which never reach cruise, so acceleration — not top speed — is what shortens them. If a held piece slips: on a long swing (algae, base to RAISE) lower the *cruise*; on the short scoring rotations lower the *acceleration*.
- **Measuring pivot kG.** First find the balance angle: disabled, pivot in Coast (Tuner X), the through-bore angle where the claw balances straight up (≈ 33°). Then, at balance + 90° (≈ 123°, claw horizontal), creep up and down at ≈ 5°/s reading `CorAl/Pivot Volts`: kG = the average of the two magnitudes, kS = half their difference. Expect ≈ 0.3 V.
- **Vision distances** are taken by magnitude, so a pasted negative `Vision/Distance` (a station tag reads negative) cannot invert a goal.

**At-target tolerances** (not tunables — they are what "arrived" means): the elevator is **0.25"** and the CorAl pivot is **1.0°** measured on the through bore, which is 0.27" at the claw. The elevator's rest band is (friction + kG error) / (12 × kP) ≈ 0.03–0.11", so a tighter window would sit inside it and the light would flicker; it is a "pose reached" flag only, and no safety gate reads it. Because the pivot's closed loop runs on the motor sensor while its tolerance is checked on the through bore, that window also has to cover the chain's backlash — so the Superstructure's settle ends when both mechanisms are at target **or** have stopped moving, whichever comes first: once motion has ceased, the loops are holding their latched setpoints and waiting longer cannot improve the pose. A separate 0.25" threshold (half of one skipped chain tooth at the carriage) drives the elevator's out-of-sync alert.

Mechanism contact geometry (tuck / low-box limits, the corridor table) is deliberately **not** tunable — those are physical facts. The CorAl pivot's Phoenix gains (kP/kS/kV/kA) are constants applied at boot and can be tried live in Phoenix Tuner X; the elevator's Spark MAX gains *are* tunables because the elevator is the mechanism that needs on-robot calibration most.

### Calibrating the elevator height
Heights are measured from the **top of the base-stage 2×1 to the bottom of the carriage 2×1**, with the middle stage between them.

**Calibration record (September 2026).** Five tape tests (2", 6", 8", 20", 40" commanded) all fit one line: **tape = gearing-model travel + a constant**. Between the 20" and 40" tests the slope is 1.005, so the gearing model is right to within 0.5 % and the *Travel Ratio* stays at 1.0. (The tests were run on the earlier 45:1 gearing, 0.244"/rotation; the sprockets, chain and rigging are unchanged, and the 15:1 value of 0.733"/rotation follows from the same model.) The constant is not a scale error but the height the carriage sits at, in that frame, on its hard stop — the **middle-stage tube, 1.000"**; the tape readings of ≈ 0.875" were an eighth of an inch optimistic. The encoder is therefore referenced *to* the hard-stop height rather than zeroed, and the reverse soft limit sits there.

A closed-loop problem looks different from a calibration problem: the **dashboard itself** stops short of the setpoint (it should hold within ~0.1"). At kP 0.4 a 1" rest error would mean ≈ 4.8 V pushing with no motion.

1. **Measure the hard-stop height once:** disabled, carriage resting on its hard stop, tape from base-2×1 top to carriage-2×1 bottom. Enter it as *Elevator – Height At Hard Stop* (Testing tab); it applies while disabled with the carriage at the hard stop, and the encoders re-reference to it. Press *Zero Elevator* (Setup tab) whenever the carriage has been moved by hand. `Elevator/Height` must read the hard-stop height at rest — the *reads below its hard-stop height* alert fires if it reads lower.
2. Arm at 100° (Pivot Setpoint 100 → Pivot Go). Testing tab: Elevator Setpoint **20** → Elev Go. Let it settle, then read the **dashboard** height and the **tape**.
3. **Dashboard at 20.0, tape off by the same amount at 20" and 40"** → the hard-stop height is wrong; adjust it by the difference. **Tape off by a percentage** (twice as far off at 40" as at 20") → set *Elevator – Travel Ratio* = current × tape ÷ commanded; it should stay at 1.0.
4. **Dashboard itself stops short** → closed loop: fix *kG* first (`Elevator/kG From Cruise`), then raise *kP* (rest error ≈ friction volts ÷ (12 × kP) inches). Leave *kS* at 0. **Overshoot** → lower *kV Scale* first, then *kA*. All re-apply the next time the robot is disabled.
5. Repeat at **40"** — dashboard and tape should agree within ⅛" at both heights. Then run L4 (51.5") once before touching *Cruise Velocity* / *Max Acceleration*.
6. **If the elevator overshoots or hunts after a change**, work in this order: (a) run one long move each way and set kG from `Elevator/kG From Cruise`; (b) watch `Elevator/Velocity` against the commanded cruise during a long move — if the carriage is still short of cruise when the profile says it should be there, raise *kA*; (c) if it rests short of the target, raise *kP* (or fix kG); if it buzzes or wobbles at rest, lower *kP*; (d) only then touch the profile. Overshoot that grows with the move length is kA; overshoot that is the same at every target is kP/kG.
7. A climb that **stutters and slows down**, especially with the *Elevator sides out of sync* alert showing, means one side is dragging: a binding stage, or a follower controller that has been given soft limits of its own (this code deliberately gives it none — see [Elevator](#elevator-elevatorjava)). `Elevator/Left Current` climbing toward the 50 A limit during a move confirms it; a healthy climb draws a few amps.

### One-time setup on each drive station laptop
1. Install Elastic (the version shipped with the WPILib 2026 installer, or newer) and connect to the robot.
2. `File → Load Layout From Robot` (Ctrl+D) and pick `elastic-layout.json` — the robot serves it from the deploy directory over HTTP port 5800 (started in Robot.java with `WebServer.start`).
3. Every laptop now has the identical layout. **Repeat step 2 whenever a deploy changes the layout** — Elastic keeps the copy it downloaded until told otherwise.

### Alerts & notifications
Persistent problems (through bore disconnected, elevator sides out of sync, elevator zeroed with the carriage raised, a calibration edit waiting to apply, motor/through-bore disagreement, a refused or restored pivot zero, a preset outside the CAD corridors, low resting battery, PathPlanner not configured, logging to internal storage) appear in the **Alerts widget** via the WPILib Alerts API; a mid-match through-bore failure additionally fires an Elastic toast notification.

### Logging & analysis (AdvantageKit + AdvantageScope)
Logging runs through **AdvantageKit** (`Robot` extends `LoggedRobot`):

- **DriverStation data, joysticks, and console output** are captured automatically.
- **Structured outputs** are recorded every loop from `Dashboard.update()`: robot `Pose2d`, `ChassisSpeeds`, swerve module states/targets, elevator/arm positions and targets, game-piece state, the best and alignment tag IDs, and the **3D component poses** (below).
- `.wpilog` files land on a USB stick (`/U/logs`) if one is mounted on the roboRIO, otherwise `/home/lvuser/logs`, where the oldest logs are pruned to keep 100 MB free and the dashboard shows a warning (in simulation: `./logs`). Open them in **AdvantageScope**. Every logged output is also published live over NetworkTables under `/AdvantageKit` (NT4Publisher).
- **Live streaming**: AdvantageScope → *Connect to Robot* with the **RLOG** source on **port 5810** (5800 is taken by the Elastic layout server). NetworkTables live viewing works too — all dashboard topics are plain NT.
- The auto chooser is a `LoggedDashboardChooser`, so every log records which auto was selected.
- CTRE's **SignalLogger** (`.hoot` files) runs alongside for Phoenix signals and **SysId** (open hoot logs in Tuner X or convert for SysId).
- Note: full AdvantageKit **log replay** would additionally require IO-layer hardware abstraction in every subsystem — not done here; this integration provides comprehensive logging, not deterministic replay.

### 3D mechanism animation (CAD in AdvantageScope)
`Dashboard` publishes `RobotState/ComponentPoses` — a `Pose3d[]` of `{middle stage, carriage, arm}` in the robot frame (X forward, Y left, Z up) — every loop, so the elevator and arm animate in 3D from real match logs **and** in simulation. A 2D schematic (`SmartDashboard/Superstructure Mechanism`) is also published for **Glass**/AdvantageScope.

The articulated robot model ships with the repository in [advantageScopeAssets/Robot_Leviathan](advantageScopeAssets/Robot_Leviathan): `model.glb` (static chassis), `model_0.glb` (middle stage), `model_1.glb` (carriage), `model_2.glb` (arm) and a `config.json`; the STEP exports they were converted from are in `advantageScopeAssets/step-files`. To use it in AdvantageScope's **3D Field** tab (one-time, per laptop):

1. *Help → Use Custom Assets Folder* and select this repository's `advantageScopeAssets` folder (or copy `Robot_Leviathan/` into the folder opened by *Help → Show Assets Folder*).
2. Select the **Leviathan - 9143A** robot model, then bind its components to the `AdvantageKit/RealOutputs/RobotState/ComponentPoses` field. Live-over-NT and log viewing both use the same field.

The `components` order in `config.json` **must** match the `Pose3d[]` order (index 0 → middle stage, 1 → carriage, 2 → arm; `Constants.LoggingConstants`). `zeroedPosition`/`zeroedRotations` describe each part's CAD origin at rest. The mounting offsets in `Dashboard.java` (`ELEVATOR_X_OFFSET`, `ARM_PIVOT_HEIGHT`) come from the CAD pivot axis (12.01" forward, 13.875" up), while the `config.json` zeroed positions are still all zero — if a part floats or swings the wrong way in the 3D view, adjust `config.json` (or the arm's rotation sign in `Dashboard.update()`) until the model lines up.

### Glass
Everything Glass needs is already on NetworkTables: the `Field` widget (`SmartDashboard/Field`), the superstructure `Mechanism2d`, the command scheduler, all `SmartDashboard` numeric topics for plotting, and the `DriveState` struct topics from `Telemetry`.

### Simulation
`./gradlew simulateJava` starts the full robot in the WPILib **Sim GUI** with physics:

- **Drivetrain**: CTRE's high-fidelity swerve simulation (drive with a gamepad, or the keyboard mapping in `simgui-ds.json`).
- **Elevator and CorAl arm**: `ElevatorSim`/`SingleJointedArmSim` physics wired to the Spark MAX and TalonFX simulation states, so preset buttons move the mechanisms — watch them in the Mechanism2d, the 3D view, or AdvantageScope connected live.
- Vision/Limelights are absent in sim; `getPivotAngle()` automatically falls back to the simulated motor sensor.
- Caveat: REV's *simulated* MAXMotion profile advances on wall-clock time and mis-scales its feedforward, so elevator profile moves overshoot in the Sim GUI before settling. This is a simulator artifact; on the robot the carriage tracks its profile. It is also why the sequence tests use their own stepped-clock mechanism models.

### Unit tests
`./gradlew test` (also run by `build`):

- `RobotContainerTest` — the whole robot wires up in a simulated HAL and the scheduler runs. Every device is constructed, so an illegal CAN ID (Phoenix IDs are 0–62) fails here instead of crashing robot code at boot.
- `ControlsBindingTest` — the operator workflow: sticks are dead without the manual modifier, the modifier cancels a running move and releases cleanly, an early score pull does not interrupt a move and fires when the gate opens, and score with no scoring goal does nothing.
- `ElevatorSimTest` — the elevator physics sim drives the carriage and the stick-release hold latches on the controller.
- `SuperstructureCorridorTest`, `SuperstructureClampSimTest`, `SuperstructureSequenceSimTest` — the clearance table, the two clamps, and every real command sequence on a stepped clock (see [Superstructure](#superstructure-superstructurejava)).
- `VisionGeometryTest` — the camera-to-robot-frame conversion for each mounting, square headings against the authored path end rotations, the tag classes, and that *Best Tag* is the closest tag any camera sees.
- `ChoreoTrajectoryTest` — the bundled Choreo trajectory loads through PathPlanner's real parser.

Two build details:

- `gradle.properties` pins Gradle to the **WPILib 2026 JDK** at its standard Windows install path (`C:/Users/Public/wpilib/2026/jdk`). Without it, Gradle uses whatever `JAVA_HOME` points at, and some other JDK distributions ship an older `msvcp140.dll` that hard-crashes WPILib's native libraries when a unit test loads them. **On macOS / Linux, or with a non-default WPILib install, change or remove that line.**
- Tests fork one JVM per class (`forkEvery`) because simulated CAN devices reject duplicate IDs within a process.

### Other tools
- **Phoenix Tuner X**: CTRE device config/firmware, hoot log viewing, swerve project generator (note: `tuner-project.json` does not match the robot — see the checklist).
- **REV Hardware Client**: Spark MAX firmware. The elevator gains are dashboard tunables, so the client is not needed for tuning.
- **PathPlanner GUI**: edit paths/autos in `src/main/deploy/pathplanner`.
- **Choreo**: draw time-optimal trajectories into `src/main/deploy/choreo`; they appear in the auto chooser automatically (see [Autonomous](#autonomous-pathplanner--choreo)).
- **Elastic**: see the Dashboard section above.
- Optional future addition: the **URCL** (Unofficial REV-Compatible Logger) vendordep would stream Spark MAX internals into AdvantageScope; AdvantageKit 26 already exposes `Logger.registerURCL(...)` for it.

---

## Autonomous (PathPlanner + Choreo)

Two path-authoring tools feed the **same** auto chooser (`SmartDashboard/Auto Mode`), so the drive team picks a PathPlanner auto **or** a Choreo trajectory from one dropdown.

**Named commands** (registered in `RobotContainer` before the autos are loaded):

| Command | What it does |
|---|---|
| `prepL4` / `prepL3` / `prepL2` | Go to the scoring pose. Use as an event marker or parallel group on the **approach path**, so the pose is reached as the robot arrives |
| `ejectCoral` | Rollers out for 0.5 s at the current pose (reversed at L1) |
| `stowAfterBackingOff` | Waits until the robot has moved 0.35 m, then stows. Put it at the **start of the departing path** |
| `intakeCoral` | Stow, then rollers until the CANrange confirms a coral; the 3 s timeout covers the roller wait only, so an empty station cannot stall the routine |
| `stow` | Stow |
| `scoreL4` / `scoreL3` / `scoreL2` | Raise, eject and stow **in place** — only with the robot clear of the reef |

Leaving L3 / L4 puts the claw 9–12" past the front bumper, so a stow in place against the reef swings it into the reef. That is why the match autos use `prep*` → `ejectCoral` → `stowAfterBackingOff` rather than the in-place `score*` commands.

### PathPlanner
- Autos live in `src/main/deploy/pathplanner/autos`, paths in `.../paths` (2025.X file format, which PathPlanner 2026 uses as well).
- **Match autos**: `Left Wall - 3 Piece` and `Right Wall - 3 Piece` — three L4 cycles, each `parallel[path, prepL4] → ejectCoral → parallel[path, stowAfterBackingOff]`, with `intakeCoral` at the station. Trim in the GUI if the full routine does not fit in 15 s at your cycle times.
- `Practice - Two Piece Front Left` / `Front Right` start at a coral station (not a legal match start) and `Test - 180 Curve` / `Test - Circle` are drivetrain checks: none of them belongs in a match.
- Every auto resets odometry to the path's starting pose (`resetOdom: true`) through `Swerve.resetPoseForAuto`. It **keeps the vision-seeded heading** (resetting only the translation) when a two-or-more-tag MegaTag1 solve was fused in the last 3 s, the estimator heading has converged on it (within 3°), and it agrees with the nominal heading within 20°; otherwise the full nominal pose is used. `Vision/Auto Kept Heading` shows which happened. MegaTag2 trusts that heading absolutely for the whole period, so overwriting a good one with the nominal placement would bias every vision pose.
- `FlippingUtil` is set to the 2025 field (17.548 × 8.052 m) before AutoBuilder is configured — PathPlannerLib 2026 otherwise mirrors red-alliance paths, odometry resets and Choreo start poses about the 2026 field's centerline, about 1 m off.
- If `settings.json` cannot be loaded, the chooser offers only `None` and an error alert says so, instead of the robot program crashing on the chooser build.
- Path constraints: 3 m/s, 4 m/s² (robot max is ~5.96 m/s).
- Path-following feedback gains live in `Constants.AutoConstants` (translation/rotation kP = 5.0). PathPlanner supplies feedforward from the path; these PIDs correct pose error.
- `FollowPathCommand.warmupCommand()` is scheduled at startup so the first path of auto starts without a stutter.
- `deleteOldFiles = true` in build.gradle removes stale paths from the roboRIO on deploy.

### Choreo
- Choreo trajectories live in `src/main/deploy/choreo` (`*.traj`). `RobotContainer.addChoreoAutos()` discovers every `.traj` at startup and adds it to the chooser as **"Choreo: &lt;name&gt;"** — exactly how PathPlanner autos are picked up from their folder. Draw a trajectory in the [Choreo](https://choreo.autos) app (create/open a project whose folder is `src/main/deploy/choreo`) and it appears automatically; a bad file is logged and skipped, never crashing robot code.
- **Following is done by PathPlanner**, not ChoreoLib: PathPlanner 2026 natively loads Choreo `.traj` files (`PathPlannerPath.fromChoreoTrajectory`), so Choreo autos run through the same `AutoBuilder` holonomic controller and `AutoConstants` gains as PathPlanner autos — one code path to tune. Each Choreo auto resets odometry to the trajectory's (alliance-flipped) start pose, then follows.
- **Why not the ChoreoLib vendordep?** At the time of writing ChoreoLib's latest release targets 2025 (`frcYear: 2025`), and GradleRIO 2026 rejects a mismatched-year vendordep outright, so importing the `.traj` through PathPlanner is the supported way to run Choreo on a 2026 project. Swap to ChoreoLib's own `AutoFactory` if a 2026 release ships.
- `Demo.traj` is a placeholder straight-line move (2 m → 3 m on X) so the feature is testable out of the box (`ChoreoTrajectoryTest` loads it through the real parser). Replace it with real trajectories.

---

## Configuration

Constants are grouped by subsystem in [Constants.java](src/main/java/frc/robot/Constants.java): `ElevatorConstants`, `CorAlConstants`, `SuperstructureConstants`, `DriveConstants`, `AutoConstants`, `VisionConstants`, `LoggingConstants` (plus the commented-out `LEDConstants`), with a CAN ID / DIO map at the top of the file. Every value is commented with its unit; values marked **TUNE** are starting points and values marked **VERIFY** are physical measurements to confirm on the robot.

Swerve hardware constants are in [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java), in CTRE Tuner X's generated format, with the per-module steer ratios set by hand for the mixed MK4i / MK4n modules.

---

## Geometry confirmed from the CAD

STEP exports of the full robot assembly (`9143-2025-A-0000 Leviathan`) and its Drivetrain / Elevator / CorAl / Funnel sub-assemblies were resolved with a small assembly walker (part placements, bounding boxes and cylinder axes, validated on known tube sizes). CAD frame: X = robot left, Y = up, Z = forward; the floor is Y = 0 (wheel centers sit at exactly 2.000"). What it settled:

| Item | CAD | Code |
|---|---|---|
| Bumper / frame envelope | 36.5" square bumpers on a 30.0" frame, centered on the origin | matches `TunerConstants` and the flush distances |
| Swerve steering axes | all four at 12.375" from center — **2.625" inset for the MK4n too**, not just the MK4i | `TunerConstants` ±12.375" ✔ |
| Elevator sprocket | "25 Chain 22t .500 Hex Sprocket" | 22T ✔ |
| Elevator hard-stop height | base bottom cross tube top 5.875", middle-stage 1" tube on it, carriage tube bottom 6.875" → **1.000"** | **1.000"**, also confirmed on the robot; the clearance map is computed at the same rest height |
| CorAl pivot axis (hard stop) | through-bore bore: **13.875" up, 12.01" forward** of center | `Dashboard` pivot height 0.352 m, X offset 0.305 m |
| Arm reach | far intake roller axes 13.9" from the pivot; at 0° the arm points 33° past vertical toward the *rear* (the intake pose, facing the funnel) | `Dashboard` arm length 0.352 m; the 33° pivot balance-angle estimate |
| Reef Limelight | lens 11.25" left, 15.78" up, 11.04" forward; 20° down, 30° toward the centerline | `LIMELIGHT_POSES[2]`, pushed to the camera |
| Funnel Limelight | mount centered, ~30" up, 11.6–14.7" behind center | consistent with the hand-measured pose |
| Barge Limelight | mount only, no camera body in the CAD | placeholder, `measured = false` |

Two things the CAD shows that the code should know about:

- The CAD shows **constant-force springs** on the middle stage. **Those are not on the robot**, so the elevator has no gravity assist and kG is the full-weight value. (The carriage stage is rigged with 4 mm Dyneema over pulleys.)
- There is a **CANivore** in the brainpan, while `TunerConstants` puts the drivetrain on the roboRIO bus (empty bus name). That is correct while the CANivore is unused; if the swerve devices are ever wired to it, the bus name must be set.

### CAD clearance analysis (claw vs. elevator)

The elevator tube contacts were mapped from the CAD instead of guessed. **Method:** every CorAl part's B-rep edges were sampled (0.2"), projected to the side view about the pivot and filled to the part's true outline; the base stage, middle stage, funnel, frame and bumpers became obstacles (tubes as exact boxes, everything else as dilated edge samples), each tested only against claw parts it overlaps in X; the middle stage rises at half the carriage travel. The map below is the model's clearance for every (arm angle, carriage height): `#` under ¼", `x` under ½", `.` under 1", blank ≥ 1". It reproduces both contacts measured on the robot (0° at ≈10.75", 5° at ≈23") within ½", so treat its figures as *real minus 0 to ½"*.

```
      carriage height (preset frame), one column per 2": 1" ... 53"
      1   9   17  25  33  41  49
    0° |x.   ####.##########x      |
    5° |x.   ##x..x########x       |
   10° |x.        .x######x        |
   15° |x.         .######.        |
   20° |x.         .####x.         |
   25° |x.         .###           x|
   30° |x.         ###.          .#|
   35° |x.         ###          .##|
   40° |x.        x##x          ###|
   45° |x.        ###.         .###|
   50° |x.       .##x         .####|
   55° |x.       .##.         #####|
   60° |x.       xx.         x#####|
   65° |x.       ..         .######|
   70° |x.                  x######|
   75° |x.                 .#######|
   80° |x.                 x######x|
   85° |x.                .######x.|
   90° |#.                .####x.. |
   95° |#.                ......   |
  100° |#.                         |
  105° |#.                         |
  110° |x.                         |
  115° |xx                         |
  120° |xx                         |
  125° |x#                         |
  130° |##                         |
  135° |##                         |
  140° |##.                        |
  145° |##.                        |
  150° |###                        |
  155° |###                        |
  160° |###.                       |
```

Reading it:

- The blocked wedge from 18–30" for angles under 65° is **band A**: the middle-stage top tube passing the claw's top.
- The blocked region above ≈36–46" for angles 25–95° is **band B**: the tube and the spring-mount hardware meeting the claw's lower rear. That is the bar an L4 pose above 22.5° hits, and the reason 90° is not a travel angle. The springs themselves are not installed; if their brackets were removed too, this band is looser than the map says — the map is the conservative case.
- The only clear columns at the top of travel are ≤ 22.5° and ≥ 97°.
- The bottom-left corner (angles > 130° below ≈7") is the bumper.

`SuperstructureConstants.FREE_CORRIDORS` is this map with a 1" margin, and the staged sequences were simulated against the full map. Two things are **not in the model**: the funnel's sheet-metal lips below ≈ 22" (the reason `ARM_RELEASE_MIN_HEIGHT` is 24") and the reef. At the L4 angle the claw stays inside the bumper plane (the game manual gives branch tips at 31.875"/47.625"/72", inset 1⅝"/1⅝"/1⅛" from the reef face), so only the coral itself reaches the branch.

The analysis scripts and the full-assembly STEP exports are **not part of this repository**. (The four STEP files under `advantageScopeAssets/step-files` are the separate per-component exports used for the AdvantageScope model.) If the claw, the elevator tubes or the spring hardware change, the analysis has to be repeated with the method above and `FREE_CORRIDORS` updated; `SuperstructureCorridorTest` will then check the presets and staged waypoints against the new table.

---

## Pre-Competition Checklist

The motion profiles, the controller mapping, reef and coral-station alignment, the gated score with auto-home, the algae hold and the L1 eject have all been run on the robot. What remains open:

1. **Measure the barge Limelight's mounting pose.** `VisionConstants.LIMELIGHT_POSES[1]` is a placeholder with `measured = false`, so barge / processor alignment supplies nothing until it is measured. Then tune *Vision – Barge Score Distance* (1.2 m) and *Processor Distance* (0.55 m), which are first guesses: push the robot into position and copy `Vision/Distance`.
2. **Set the elevator kG from `Elevator/kG From Cruise`** (one long move each way from the Testing tab, arm at 100°). It ships at the CAD estimate, 1.0 V.
3. **Measure the pivot kG and balance angle** (*Pivot – kG* ships at 0, the 33° balance angle is an estimate) — procedure under the tunables table. Until then the arm rests 2–3° low at RAISE and the algae poses.
4. **Do not lower `ARM_RELEASE_MIN_HEIGHT` (24") without a jog check.** The funnel sheet metal below ≈ 22" is not in the CAD clearance model — see [Superstructure](#superstructure-superstructurejava).
5. **Confirm the L1 eject speed.** `CORAL_L1_SCORE_SPEED` (−0.3) is a starting value; L1 ejects against the intake direction.
6. **Re-check the vision goal tunables on a real field.** Push the robot flush on the reef base and copy `Vision/Distance` into *Vision – Reef Flush Distance*; center it on a branch and copy the magnitude of `Vision/Lateral` into *Reef Branch Offset*. For the station, the funnel camera loses the tag about 0.55 m from the wall (its +50° pitch), so keep *Station Flush Distance* at the geometric 0.47 m — the tracker finishes the last stretch on odometry. All readouts are robot-frame, so the camera that sees the tag does not matter. If an axis ever drives the wrong way, the camera pose (not a sign flag) is what to check, in the camera's web-UI 3D preview.
7. **Verify a 20" and a 40" elevator move with a tape** after any mechanical work on the elevator — see [Calibrating the elevator height](#calibrating-the-elevator-height). The hard-stop height is 1.000" and the travel ratio should stay at 1.0.
8. **Sanity-check the CorAl pivot angle** against the through bore encoder after any work on the pivot. The 65.41:1 ratio is derived from the real gear train (10:58 → 18:58 → 12:42), and the through bore must be mounted 1:1 on the pivot shaft.
9. **Re-check the coral's landing on each branch** if a preset is changed. Every preset is inside the CAD corridors with a full inch of model clearance: L3 is (30.5", 25°) — 27.5° at the same height has 1¾" if it ever rubs — and L2 is 12.5° rather than the geometric 5° so the arm keeps an inch to the cross bar even when chain slack lets it sag 2.5°.
10. **CANcoder offsets: do not regenerate `TunerConstants.java` from `tuner-project.json`.** The Tuner project models all four modules as MK4n and its offsets disagree with `TunerConstants.java`, which holds the values the robot runs with (mixed per-module steer ratios, offsets, 12.375" module positions confirmed from the CAD). After any module work, re-measure the offsets in Tuner X with the wheels aligned straight forward and enter them by hand.
11. **Firmware**: 2026 firmware on all CTRE devices (TalonFX, CANcoder, Pigeon 2, CANrange), current Spark MAX firmware via the REV Hardware Client, the 2026 roboRIO image, Limelight OS 2026.0+.
12. **CAN IDs** must match the map at the top of `Constants.java` (the CANrange is ID 3). Phoenix device IDs must be 0–62.
13. **Reload the Elastic layout** on every drive laptop after deploying (`File → Load Layout From Robot`).
14. **Raise the *Drive – Teleop Speed Scale* tunable** on the Testing tab (0.25 for indoor testing) as the drivers are ready — no redeploy needed. Tune `AutoConstants` path-following kP and the autos' timing on a full field.
15. **LEDs**: skip while the CANdle code is commented out. When one is installed, un-comment the subsystem and set `LEDConstants.LED_COUNT` to the actual LED strip length.

---

## Dependencies

- **WPILib 2026** (GradleRIO 2026.2.1, Java 17): the core library for FRC robot programming.
- **Phoenix 6 (26.3.0)**: CTRE motor controllers (Kraken X60/TalonFX) and sensors (CANcoder, CANrange, Pigeon 2; CANdle when installed).
- **REVLib 2026.0.5**: Spark MAX controllers and NEO motors on the elevator.
- **PathplannerLib 2026.1.2**: autonomous path generation and following — also loads **Choreo** `.traj` files, so Choreo is supported with no extra vendordep.
- **AdvantageKit 26.0.2**: logging framework (`LoggedRobot`, `.wpilog` + RLOG live stream, logged auto chooser).
- **LimelightHelpers v1.14** (source file in the project): Limelight interface; requires Limelight OS 2026.0+ on the cameras.

## Getting Started

1. Install the **WPILib 2026** tools, then **clone this repository**.
2. **Open the project** in WPILib VS Code 2026. On macOS / Linux, or with a non-default WPILib install, first point `org.gradle.java.home` at your WPILib JDK — either edit the line in `gradle.properties`, or override it in `~/.gradle/gradle.properties`, which takes precedence and leaves the repository untouched (see [Unit tests](#unit-tests)).
3. Set your team number in `.wpilib/wpilib_preferences.json` if you are not Team 9143.
4. **Build and test** with `./gradlew build`; try it without a robot with `./gradlew simulateJava`.
5. **Deploy** (`./gradlew deploy` or the WPILib "Deploy Robot Code" command), then load the dashboard layout: Elastic → `File → Load Layout From Robot`.
6. Before driving, zero the mechanisms with both at their base (operator Back / Start held 1 s while disabled, or the Setup tab buttons) and work through the [Pre-Competition Checklist](#pre-competition-checklist).
