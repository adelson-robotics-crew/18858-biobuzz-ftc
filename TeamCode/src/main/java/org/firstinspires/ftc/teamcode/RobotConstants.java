package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/**
 * Definitions only, no logic: hardware names, gamepad button mapping, and tunable robot numbers.
 * Robot.java reads the gamepad through the mapping below; subsystems read their hardware names and tunables from here.
 * Pedro's own tuned values (PIDs, pod offsets, drivetrain motor names, etc.) stay in subsystems/drivetrain/pedro/Constants.java.
 */
public final class RobotConstants {

    // =====================================================================================
    // HARDWARE NAMES
    // Must match the names in the Robot Controller's hardware configuration.
    // =====================================================================================
    // Intake
    public static final String RIGHT_INTAKE_SERVO_NAME = "intake_right"; // continuous-rotation servo
    public static final String LEFT_INTAKE_SERVO_NAME = "intake_left";   // continuous-rotation servo
    public static final String INTAKE_MOTOR_NAME = "intake";             // DC motor

    // Shooter
    public static final String SHOOTER_MOTOR_NAME = "shooter";           // DC motor
    public static final String INDEXER_SERVO_NAME = "indexer";           // continuous-rotation servo

    // =====================================================================================
    // GAMEPAD BUTTON MAPPING
    // Every control on the driver gamepad (gamepad1). Each constant reads one gamepad field
    // directly, so to remap a control, change its line here. Nothing else needs to change.
    // =====================================================================================

    // Driving. Each SIGN is multiplied into the stick value: -1.0 flips it, 1.0 leaves it alone.
    public static final ToDoubleFunction<Gamepad> DRIVE_FORWARD_AXIS = gamepad -> gamepad.left_stick_y;
    public static final double DRIVE_FORWARD_AXIS_SIGN = -1.0; // a stick reads negative when pushed up, so flip it to make up = forward

    public static final ToDoubleFunction<Gamepad> DRIVE_STRAFE_AXIS = gamepad -> gamepad.left_stick_x;
    public static final double DRIVE_STRAFE_AXIS_SIGN = -1.0;  // inverted for this robot

    public static final ToDoubleFunction<Gamepad> DRIVE_TURN_AXIS = gamepad -> gamepad.right_stick_x;
    public static final double DRIVE_TURN_AXIS_SIGN = -1.0;    // inverted for this robot

    // Mechanisms (hold to run; the robot won't run the intake and shooter at the same time)
    // Triggers are analog (0 to 1), so a trigger counts as pressed once it's pulled past TRIGGER_PRESSED_THRESHOLD.
    public static final double TRIGGER_PRESSED_THRESHOLD = 0.5;
    public static final Predicate<Gamepad> INTAKE_BUTTON = gamepad -> gamepad.right_trigger > TRIGGER_PRESSED_THRESHOLD;
    // Spins the flywheel; the indexer feeds automatically whenever the wheel is up to speed
    public static final Predicate<Gamepad> SHOOT_BUTTON = gamepad -> gamepad.left_trigger > TRIGGER_PRESSED_THRESHOLD;

    // Shooter RPM tuning on the fly. Each press (not hold) bumps the shooter target RPM by SHOOTER_RPM_ADJUST_STEP.
    public static final Predicate<Gamepad> SHOOTER_RPM_UP_BUTTON = gamepad -> gamepad.right_bumper;
    public static final Predicate<Gamepad> SHOOTER_RPM_DOWN_BUTTON = gamepad -> gamepad.left_bumper;

    // Auto-shoot: one press drives to the shooting pose and shoots. The OpMode supplies the shooting pose and
    // the rest of the routine's numbers through Robot.configureAutoShoot(); if it doesn't, the button does nothing.
    // Pressing it again while the routine is running cancels it and hands control back to the driver.
    public static final Predicate<Gamepad> AUTO_SHOOT_BUTTON = gamepad -> gamepad.x;

    // Zero pose: with the robot in the bottom left corner, one press sets the localizer to the zero pose
    // (see the Zero pose constants below). Also cancels the auto-shoot routine if it's running.
    public static final Predicate<Gamepad> ZERO_POSE_BUTTON = gamepad -> gamepad.b;

    // =====================================================================================
    // TUNABLE ROBOT CONSTANTS
    // =====================================================================================

    // Field positions
    // Pedro coordinates: x and y in inches, heading in degrees (counterclockwise is positive).
    // x = forward and y = left, as seen from the starting pose.

    // Where the robot is at INIT: always placed in the bottom right corner, facing forward.
    // The TeleOp sets the localizer to this pose, so the field positions below are measured from here.
    public static final double START_POSE_X_INCHES = 0.0;
    public static final double START_POSE_Y_INCHES = 0.0;
    public static final double START_POSE_HEADING_DEGREES = 0.0;

    // What the localizer is set to when ZERO_POSE_BUTTON is pressed with the robot in the bottom left corner.
    public static final double ZERO_POSE_X_INCHES = 0.0;
    public static final double ZERO_POSE_Y_INCHES = 0.0;
    public static final double ZERO_POSE_HEADING_DEGREES = 0.0;

    // Driver input

    // Stick values with a magnitude at or below this are treated as 0.
    // 0.0 means no deadband, which is how the TeleOp behaved before this constant existed.
    public static final double STICK_DEADBAND = 0.0;

    // Drivetrain

    // A drive command (follow a path, hold a pose) counts as finished once the robot is within
    // these distances of its target pose, in both position and heading.
    public static final double DRIVE_POSITION_TOLERANCE_INCHES = 1.0;
    public static final double DRIVE_HEADING_TOLERANCE_DEGREES = 3.0;

    // Heading lock (TeleOp). While the turn stick is at or below HEADING_LOCK_TURN_THRESHOLD, the robot holds
    // its heading instead of drifting; pushing the stick past it turns the lock off so the driver turns freely.
    // Set HEADING_LOCK_ENABLED to false to drive exactly as before the lock existed.
    public static final boolean HEADING_LOCK_ENABLED = false;
    // Turn stick magnitude (0 to 1) at or below which the stick counts as released and the lock engages
    public static final double HEADING_LOCK_TURN_THRESHOLD = 0.05;
    // After the turn stick is released, the lock waits until the robot is rotating slower than this before it
    // grabs the heading to hold. Grabbing it right away would pull the robot back against its own turning momentum.
    public static final double HEADING_LOCK_SETTLE_DEGREES_PER_SECOND = 20.0;
    // PD gains of the heading lock: turn power per radian of heading error, and turn power per rad/s of rotation
    // (the D term damps the correction so it doesn't overshoot). Raise P if the robot gets knocked off its
    // heading too easily; raise D if it wobbles back and forth around the locked heading.
    public static final double HEADING_LOCK_P = 0.2;
    public static final double HEADING_LOCK_D = 0.05;

    // Intake

    // The two intake servos face opposite ways, so pulling a ball in needs opposite signs.
    // If a servo pushes the ball out instead of in, flip that servo's sign.
    public static final double INTAKE_RIGHT_SERVO_POWER = -1.0;
    public static final double INTAKE_LEFT_SERVO_POWER = 1.0;
    public static final double INTAKE_MOTOR_POWER = 1.0;

    // Shooter

    // Starting speed the shooter wheel is held at while shooting, in RPM of the motor's output shaft.
    // The driver can bump it up/down during TeleOp (SHOOTER_RPM_UP_BUTTON / SHOOTER_RPM_DOWN_BUTTON); once a
    // good value is found from telemetry, copy it here.
    // Held with setVelocity(), so it stays the same as the battery drains. Keep it comfortably below
    // the motor's free speed (6000 RPM) so the controller has headroom on a low battery.
    // 2500 is (rounded) the same wheel speed the code used to call "130 RPM": that label came from the old,
    // wrong ticks-per-rev (537.7), which made every reading 537.7 / 28 = 19.2x too low. 130 * 19.2 = 2496.4.
    public static final double SHOOTER_TARGET_RPM = 2500.0;

    // How much one press of SHOOTER_RPM_UP_BUTTON / SHOOTER_RPM_DOWN_BUTTON changes the target RPM.
    public static final double SHOOTER_RPM_ADJUST_STEP = 50.0;

    // The adjusted target RPM is kept within these limits. The max is the motor's free speed; above it
    // the wheel can never reach the target, so the indexer would never feed.
    public static final double SHOOTER_MIN_TARGET_RPM = 0.0;
    public static final double SHOOTER_MAX_TARGET_RPM = 6000.0;

    // While SHOOT_BUTTON is held, the indexer feeds balls whenever the shooter wheel is no more than this many
    // RPM below the current target RPM, and stops again whenever the wheel drops further (or SHOOT_BUTTON is released).
    public static final double INDEXER_RPM_BELOW_TARGET = 40.0;
    // Once the wheel gets within INDEXER_RPM_BELOW_TARGET, it must stay there this long before the indexer starts,
    // so it has time to finish spinning up. If the wheel drops out of range, the wait starts over the next time it's back.
    public static final double INDEXER_FEED_DELAY_MILLISECONDS = 500.0;
    public static final double INDEXER_SERVO_POWER = -0.5; // feeds balls into the shooter wheel (half speed)

    // Velocity PIDF gains the motor controller uses to hold the shooter wheel at the target RPM (setVelocity()).
    // While SHOOTER_USE_CUSTOM_VELOCITY_PIDF is false, the controller keeps its default gains and these four
    // numbers are ignored. The values below are the REV hub firmware's defaults (the goBILDA motor type in the
    // SDK doesn't set its own), so turning the flag on with them unchanged behaves the same as leaving it off.
    // Units are the hub's own (error in encoder ticks per second), not RPM.
    // Overshoot/oscillation: lower P, raise D, and also check I (it keeps pushing until the error is gone, which
    // adds overshoot on a heavy wheel). F is a feedforward: power applied in proportion to the target speed.
    public static final boolean SHOOTER_USE_CUSTOM_VELOCITY_PIDF = true;

    public static final double SHOOTER_VELOCITY_F = 13.0;
    public static final double SHOOTER_VELOCITY_P = 100.0;
    public static final double SHOOTER_VELOCITY_I = 0.00;
    public static final double SHOOTER_VELOCITY_D = 0.0;

    // Encoder ticks per one revolution of the shooter motor's output shaft.
    // The shooter is a goBILDA 5203 Yellow Jacket 6000 RPM (5203-2402-0001, 1:1, no gearbox), so the encoder's
    // 28 ticks per turn of the motor shaft are also 28 ticks per turn of the output shaft.
    public static final double SHOOTER_ENCODER_TICKS_PER_REV = 28.0;

    /**
     * Not meant to be instantiated; this class only holds constants.
     */
    private RobotConstants() {
    }
}
