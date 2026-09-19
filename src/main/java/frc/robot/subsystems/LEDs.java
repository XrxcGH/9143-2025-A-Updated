// ============================================================================
// CANdle LED subsystem - commented out (Sept 2026): there is no CANdle on the
// robot, so every line below is disabled to keep the device off the CAN bus
// and the class out of the build. To bring it back: remove the leading "// "
// from every line of this file, uncomment LEDConstants and its two imports in
// Constants.java, and uncomment the lines marked "CANdle disabled" in
// RobotContainer.java and Dashboard.java (which then publishes the real
// "LEDs/State").
// ============================================================================

// package frc.robot.subsystems;
// 
// import java.util.function.BooleanSupplier;
// 
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.EmptyAnimation;
// import com.ctre.phoenix6.controls.LarsonAnimation;
// import com.ctre.phoenix6.controls.RainbowAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;
// 
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// 
// import frc.robot.Constants.LEDConstants;
// 
// /**
//  * LED subsystem driven by a CTRE CANdle (Phoenix 6).
//  *
//  * The LEDs communicate robot state to the drive team and field staff at a
//  * glance. State is derived automatically every loop from the robot mode and
//  * from suppliers wired in from the other subsystems - no commands need to be
//  * scheduled for the LEDs to work, and nothing else in the robot code has to
//  * know the LEDs exist.
//  *
//  * State priority (highest first):
//  *   1. DISABLED        - alliance-colored Larson ("scanner") animation
//  *   2. AUTONOMOUS      - rainbow (obvious "hands off, robot is in control")
//  *   3. HAS_GAME_PIECE  - solid green (drivers: you can score / leave the station)
//  *   4. VISION_TRACKING - strobing cyan (robot is auto-aligning; expect motion)
//  *   5. ENDGAME         - strobing yellow (last N seconds of the match)
//  *   6. TELEOP_IDLE     - solid alliance color
//  *
//  * Control requests are only re-sent when the state changes, so this subsystem
//  * adds essentially zero CAN bus traffic during steady-state operation.
//  */
// public class LEDs extends SubsystemBase {
// 
//     /** All the visual states the LEDs can be in, in priority order. */
//     public enum LEDState {
//         DISABLED,
//         AUTONOMOUS,
//         HAS_GAME_PIECE,
//         VISION_TRACKING,
//         ENDGAME,
//         TELEOP_IDLE
//     }
// 
//     // Colors, animation rates and the strip type are LEDConstants.
// 
//     private final CANdle candle;
// 
//     // State inputs from the rest of the robot
//     private final BooleanSupplier hasGamePiece;    // e.g., CorAl CANrange detection
//     private final BooleanSupplier visionTracking;  // e.g., Swerve AprilTag tracking enabled
// 
//     // Reusable control requests (allocated once; sent only on state change).
//     // Index range covers the CANdle's 8 onboard LEDs plus the attached strip.
//     private final SolidColor solidRequest =
//         new SolidColor(0, LEDConstants.LED_COUNT - 1);
//     private final StrobeAnimation strobeRequest =
//         new StrobeAnimation(0, LEDConstants.LED_COUNT - 1).withSlot(0).withFrameRate(LEDConstants.STROBE_FRAME_RATE);
//     private final LarsonAnimation larsonRequest =
//         new LarsonAnimation(0, LEDConstants.LED_COUNT - 1).withSlot(0)
//             .withSize(LEDConstants.LARSON_SIZE).withFrameRate(LEDConstants.LARSON_FRAME_RATE);
//     private final RainbowAnimation rainbowRequest =
//         new RainbowAnimation(0, LEDConstants.LED_COUNT - 1).withSlot(0).withFrameRate(LEDConstants.RAINBOW_FRAME_RATE);
//     // Clears whatever animation occupies slot 0 (needed before showing a solid color)
//     private final EmptyAnimation clearAnimationRequest = new EmptyAnimation(0);
// 
//     /** Currently displayed state; null forces the first periodic() to apply one. */
//     private LEDState currentState = null;
// 
//     /** Alliance color in the currently displayed pattern (the DS may report
//      *  the alliance late, so an alliance change must re-apply the pattern). */
//     private RGBWColor displayedAllianceColor = null;
// 
//     /**
//      * @param hasGamePiece   supplier that is true while a game piece is held
//      * @param visionTracking supplier that is true while vision auto-alignment is active
//      */
//     public LEDs(BooleanSupplier hasGamePiece, BooleanSupplier visionTracking) {
//         this.hasGamePiece = hasGamePiece;
//         this.visionTracking = visionTracking;
// 
//         candle = new CANdle(LEDConstants.CANDLE_ID);
// 
//         // Configure the strip type (LEDConstants.STRIP_TYPE: change it if
//         // colors appear swapped on the actual hardware) and global brightness.
//         CANdleConfiguration config = new CANdleConfiguration();
//         config.LED.StripType = LEDConstants.STRIP_TYPE;
//         config.LED.BrightnessScalar = LEDConstants.BRIGHTNESS;
//         candle.getConfigurator().apply(config);
//     }
// 
//     /**
//      * Decides which state should be displayed right now, in priority order.
//      * Runs every loop; cheap boolean checks only.
//      */
//     private LEDState computeState() {
//         if (DriverStation.isDisabled()) {
//             return LEDState.DISABLED;
//         }
//         if (DriverStation.isAutonomousEnabled()) {
//             return LEDState.AUTONOMOUS;
//         }
//         if (hasGamePiece.getAsBoolean()) {
//             return LEDState.HAS_GAME_PIECE;
//         }
//         if (visionTracking.getAsBoolean()) {
//             return LEDState.VISION_TRACKING;
//         }
//         // getMatchTime() counts down within the current period during a real
//         // match and returns -1 in practice mode, so this only fires on a field.
//         double matchTime = DriverStation.getMatchTime();
//         if (DriverStation.isTeleopEnabled() && matchTime > 0
//             && matchTime <= LEDConstants.ENDGAME_WARNING_TIME) {
//             return LEDState.ENDGAME;
//         }
//         return LEDState.TELEOP_IDLE;
//     }
// 
//     /** The alliance color, or purple when the alliance is not yet known. */
//     private RGBWColor allianceColor() {
//         return DriverStation.getAlliance()
//             .map(alliance -> alliance == Alliance.Red
//                 ? LEDConstants.RED_ALLIANCE_COLOR : LEDConstants.BLUE_ALLIANCE_COLOR)
//             .orElse(LEDConstants.NO_ALLIANCE_COLOR);
//     }
// 
//     /**
//      * Sends the control requests for a state to the CANdle. Called only on
//      * state transitions. Solid colors clear the animation slot first so a
//      * previous animation doesn't keep overwriting the LEDs.
//      */
//     private void applyState(LEDState state) {
//         switch (state) {
//             case DISABLED:
//                 candle.setControl(larsonRequest.withColor(allianceColor()));
//                 break;
//             case AUTONOMOUS:
//                 candle.setControl(rainbowRequest);
//                 break;
//             case HAS_GAME_PIECE:
//                 candle.setControl(clearAnimationRequest);
//                 candle.setControl(solidRequest.withColor(LEDConstants.GAME_PIECE_COLOR));
//                 break;
//             case VISION_TRACKING:
//                 candle.setControl(strobeRequest.withColor(LEDConstants.TRACKING_COLOR));
//                 break;
//             case ENDGAME:
//                 candle.setControl(strobeRequest.withColor(LEDConstants.ENDGAME_COLOR));
//                 break;
//             case TELEOP_IDLE:
//             default:
//                 candle.setControl(clearAnimationRequest);
//                 candle.setControl(solidRequest.withColor(allianceColor()));
//                 break;
//         }
//     }
// 
//     /** The state currently being displayed (for dashboards/testing). */
//     public LEDState getState() {
//         return currentState;
//     }
// 
//     @Override
//     public void periodic() {
//         LEDState desired = computeState();
//         RGBWColor alliance = allianceColor();
// 
//         // Re-apply when the state changes, or when an alliance-colored state
//         // is showing and the reported alliance changes (e.g., the Driver
//         // Station connected after startup). Reference comparison is safe
//         // because allianceColor() returns shared constants.
//         boolean allianceChanged =
//             (desired == LEDState.DISABLED || desired == LEDState.TELEOP_IDLE)
//             && alliance != displayedAllianceColor;
// 
//         if (desired != currentState || allianceChanged) {
//             applyState(desired);
//             currentState = desired;
//             displayedAllianceColor = alliance;
//         }
//     }
// }
