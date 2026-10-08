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
    public static final String INDEXER_SERVO_NAME = "indexer";           // Axon servo in continuous mode (configure as a Continuous Rotation Servo)
    public static final String INDEXER_ENCODER_NAME = "indexer_encoder"; // the Axon's position feedback wire, on analog port 0 (configure as an Analog Input)

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
    public static final Predicate<Gamepad> INTAKE_BUTTON = gamepad -> gamepad.left_trigger > TRIGGER_PRESSED_THRESHOLD;
    // Manual shot: spins the flywheel at the fixed SHOOTER_TARGET_RPM (not the shot table, so it still works if the
    // robot's position is off) and the indexer feeds automatically whenever the wheel is up to speed. The driver
    // drives to the usual spot and shoots from there.
    public static final Predicate<Gamepad> SHOOT_BUTTON = gamepad -> gamepad.right_trigger > TRIGGER_PRESSED_THRESHOLD;

    // Shooter RPM tuning on the fly. Each press (not hold) bumps the shooter target RPM by SHOOTER_RPM_ADJUST_STEP.
    // Off for matches, so a bumper pressed by accident can't change the shooting speed. Turn on to tune RPM.
    public static final boolean SHOOTER_RPM_ADJUST_ENABLED = false;
    public static final Predicate<Gamepad> SHOOTER_RPM_UP_BUTTON = gamepad -> gamepad.right_bumper;
    public static final Predicate<Gamepad> SHOOTER_RPM_DOWN_BUTTON = gamepad -> gamepad.left_bumper;

    // Aim and shoot (hold): the robot turns to face the shooter at a HIVE cell while the left stick still drives,
    // the shooter spins at the shot table's RPM for the current distance, and the indexer feeds only while the
    // position is valid (distance and angle), the heading is within HEADING_TOLERANCE_DEG, and the wheel is at speed.
    // The turn stick is ignored while held. Release to go back to normal driving; the shooter stops.
    // Left and right are as the driver sees them; which cell each one is depends on the alliance (see the TeleOp).
    // If both are held, left wins.
    public static final Predicate<Gamepad> AIM_LEFT_CELL_BUTTON = gamepad -> gamepad.x;
    public static final Predicate<Gamepad> AIM_RIGHT_CELL_BUTTON = gamepad -> gamepad.b;

    // Emergency heading reset (one press): with the robot facing straight away from the driver (+x, the way the
    // forward stick drives), sets the heading to 0 deg. In Match TeleOp x and y are left alone; it's only for when the
    // heading has drifted or the robot got spun. In DriveTest it resets the whole pose to (0, 0, 0 deg), so put the
    // robot back in the practice corner, facing away from the driver, before pressing it.
    // "back" is the small button on the left side above the mode button (labeled Share on PlayStation controllers).
    public static final Predicate<Gamepad> RESET_HEADING_BUTTON = gamepad -> gamepad.back;

    // =====================================================================================
    // TUNABLE ROBOT CONSTANTS
    // =====================================================================================

    // Field positions
    // Everything uses Pedro field coordinates: x and y in inches, heading in degrees (counterclockwise is positive).
    // Each OpMode sets its own starting pose (the autos in their routine, the TeleOps in their red/blue wrapper).

    // Driver input

    // Stick values with a magnitude at or below this are treated as 0.
    // 0.0 means no deadband, which is how the TeleOp behaved before this constant existed.
    public static final double STICK_DEADBAND = 0.0;

    // Drivetrain

    // A drive command (follow a path, hold a pose) counts as finished once the robot is within
    // these distances of its target pose, in both position and heading. The same tolerances are used at INIT to
    // check that the odometry really took the starting pose (see Drivetrain.isAtPose()).
    public static final double DRIVE_POSITION_TOLERANCE_INCHES = 1.0;
    public static final double DRIVE_HEADING_TOLERANCE_DEGREES = 3.0;

    // Stall detection (every drive command, e.g. autonomous legs). If the robot hasn't moved
    // more than DRIVE_STALL_MOVEMENT_INCHES or turned more than DRIVE_STALL_TURN_DEGREES for DRIVE_STALL_SECONDS
    // while a command is driving it, it's assumed to be pushing into a wall: the drivetrain stops and the command
    // counts as finished, as if the robot had reached its target.
    public static final double DRIVE_STALL_SECONDS = 1.0;
    public static final double DRIVE_STALL_MOVEMENT_INCHES = 0.5;
    public static final double DRIVE_STALL_TURN_DEGREES = 2.0;

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

    // Speed the shooter wheel is held at for a manual shot (SHOOT_BUTTON), in RPM of the motor's output shaft.
    // Aim-and-shoot uses the shot table instead. With SHOOTER_RPM_ADJUST_ENABLED on, the bumpers bump the manual
    // speed up/down during TeleOp; once a good value is found from telemetry, copy it here.
    // Held with setVelocity(), so it stays the same as the battery drains. Keep it comfortably below
    // the motor's free speed (6000 RPM) so the controller has headroom on a low battery.
    public static final double SHOOTER_TARGET_RPM = 2425.0;

    // How much one press of SHOOTER_RPM_UP_BUTTON / SHOOTER_RPM_DOWN_BUTTON changes the target RPM.
    public static final double SHOOTER_RPM_ADJUST_STEP = 50.0;

    // The adjusted target RPM is kept within these limits. The max is the motor's free speed; above it
    // the wheel can never reach the target, so the indexer would never feed.
    public static final double SHOOTER_MIN_TARGET_RPM = 0.0;
    public static final double SHOOTER_MAX_TARGET_RPM = 6000.0;

    // Indexer feed hysteresis. While shooting, the wheel counts as "at speed" once it gets within
    // INDEXER_START_FEED_RPM_TOLERANCE of the target RPM (above or below), and keeps counting as at speed until it
    // drifts more than INDEXER_STOP_FEED_RPM_TOLERANCE away. The gap keeps the indexer from flickering on and off
    // when the wheel sags a little as a ball goes through. The indexer feeds only while the wheel is at speed.
    public static final double INDEXER_START_FEED_RPM_TOLERANCE = 25.0;
    public static final double INDEXER_STOP_FEED_RPM_TOLERANCE = 50.0;
    // Power the indexer spins at while feeding (the sign is the feeding direction). The Axon is brushless and fast,
    // so this is kept low
    public static final double INDEXER_SERVO_POWER = -0.2;

    // Indexer rest positions. Whenever the indexer isn't feeding, it turns to the nearest rest angle and holds there
    // so balls can't feed through. Angles are measured from where the indexer is at INIT (that's 0 deg, so put it in
    // a rest position before pressing INIT); there's a rest angle every INDEXER_REST_SPACING_DEG (0, 90, 180, 270).
    public static final double INDEXER_REST_SPACING_DEG = 90.0;
    // +1.0 if positive servo power makes the encoder angle go up, -1.0 if it makes it go down. If the indexer won't
    // settle at rest (it keeps spinning or runs away when it should stop), flip this.
    // -1.0: the 12:56 DriveTest log showed the angle going up while feeding at negative power. With +1.0 the rest
    // controller pushed away from rest and swung back and forth by up to 45 deg.
    public static final double INDEXER_ENCODER_DIRECTION = -1.0;
    // The Axon's feedback voltage covers one full turn (0 V to the hub's max analog voltage = 0 to 360 deg).
    public static final double INDEXER_ENCODER_DEGREES_PER_TURN = 360.0;
    // Rest-holding controller: servo power per degree off the rest angle, clamped to INDEXER_REST_MAX_POWER.
    // Within INDEXER_REST_TOLERANCE_DEG it counts as at rest and the power is 0, so the servo doesn't buzz.
    // Outside it, at least INDEXER_REST_MIN_POWER is used so friction can't stall it a few degrees short.
    // Strong enough to hold the indexer in place while the robot turns, but not so strong it wobbles. At 0.2 power
    // the Axon turns about 12 deg per loop, so the power near rest stays well below that. If it wobbles around
    // rest, lower INDEXER_REST_KP; if it gets knocked off rest too easily, raise it.
    // History: 0.005 / 0.1 held too weakly, 0.012 / 0.2 was jerky.
    public static final double INDEXER_REST_KP = 0.004;
    public static final double INDEXER_REST_MAX_POWER = 1.0;
    public static final double INDEXER_REST_MIN_POWER = 0.04;
    public static final double INDEXER_REST_TOLERANCE_DEG = 3.0;

    // Velocity PIDF gains the motor controller uses to hold the shooter wheel at the target RPM (setVelocity()).
    // While SHOOTER_USE_CUSTOM_VELOCITY_PIDF is false, the controller keeps its default gains and these four
    // numbers are ignored. The values below are the REV hub firmware's defaults (the goBILDA motor type in the
    // SDK doesn't set its own), so turning the flag on with them unchanged behaves the same as leaving it off.
    // Units are the hub's own (error in encoder ticks per second), not RPM.
    // Overshoot/oscillation: lower P, raise D, and also check I (it keeps pushing until the error is gone, which
    // adds overshoot on a heavy wheel). F is a feedforward: power applied in proportion to the target speed.
    public static final boolean SHOOTER_USE_CUSTOM_VELOCITY_PIDF = true;

    public static final double SHOOTER_VELOCITY_F = 13.0;
    public static final double SHOOTER_VELOCITY_P = 300.0;
    public static final double SHOOTER_VELOCITY_I = 0.00;
    public static final double SHOOTER_VELOCITY_D = 0.0;

    // Encoder ticks per one revolution of the shooter motor's output shaft.
    // The shooter is a goBILDA 5203 Yellow Jacket 6000 RPM (5203-2402-0001, 1:1, no gearbox), so the encoder's
    // 28 ticks per turn of the motor shaft are also 28 ticks per turn of the output shaft.
    public static final double SHOOTER_ENCODER_TICKS_PER_REV = 28.0;

    // Match Auto shooting
    // The auto shoots from a fixed spot outside the shot table's range, so it uses its own RPM instead of the table.
    public static final double AUTO_SHOOTER_TARGET_RPM = 2425.0;
    // How long the auto keeps shooting, counted from when the wheel first reaches speed (indexer feeding starts then).
    public static final double AUTO_SHOOT_SECONDS = 4.0;
    // If the wheel hasn't reached speed by this long after the shooter starts, the AUTO_SHOOT_SECONDS of shooting start anyway,
    // so a wheel that never settles can't keep the robot from parking.
    public static final double AUTO_SPIN_UP_TIMEOUT_SECONDS = 3.0;

    // Shot aiming (AIM_LEFT_CELL_BUTTON / AIM_RIGHT_CELL_BUTTON), in Pedro field coordinates.

    // THE MEASURED CELL: the BLUE hive's LOWER-RIGHT cell, as seen in the Pedro Pathing visualizer. Every shot
    // measurement and the shot table below were taken against this one cell. The other three cells are derived from
    // it by symmetry (see ShotTarget), so this is the only cell position stored.
    // The point is the least-squares intersection of all measured shot headings. It was measured as (72.95, 47.56)
    // in the old TeleOp frame, which is Pedro minus 8.5 in on x and y.
    public static final double MEASURED_CELL_X_INCHES = 81.45;
    public static final double MEASURED_CELL_Y_INCHES = 56.06;
    // Direction from the measured cell to the spot straight in front of it: it's shot from below (-y), i.e. 270 deg
    public static final double MEASURED_CELL_FRONT_DIRECTION_DEGREES = 270.0;

    // The point the field is symmetric about. The four cells are mirror images of each other around it:
    // blue lower (81.45, 56.06), blue upper (81.45, 85.44), red upper (60.05, 85.44), red lower (60.05, 56.06).
    public static final double FIELD_SYMMETRY_CENTER_X_INCHES = 70.75;
    public static final double FIELD_SYMMETRY_CENTER_Y_INCHES = 70.75;

    // Distance (inches from the target) -> shooter motor RPM, linearly interpolated between rows (see ShotTable).
    // Distances must be ascending. The curve is deliberately U-shaped: close in, the shot needs more speed to
    // clear the front lip of the CELL. RPMs are as read from shooter telemetry (SHOOTER_ENCODER_TICKS_PER_REV).
    public static final double[] SHOT_TABLE_DISTANCES_INCHES = {42.0, 45.0, 48.0, 52.0, 62.4};
    public static final double[] SHOT_TABLE_RPMS = {2475.0, 2400.0, 2440.0, 2460.0, 2525.0};

    // A position is only valid to shoot from within this distance range (inclusive). The 42 in table row exists
    // for interpolation, but 42 in is the edge of what works, so the range starts at 43.
    public static final double SHOT_MIN_DISTANCE_INCHES = 43.0;
    public static final double SHOT_MAX_DISTANCE_INCHES = 62.4;

    // A position is also only valid within this many degrees either side of straight in front of the cell
    // (see ShotTarget.frontDirectionDegrees).
    public static final double MAX_ANGLE_OFF_CENTER_DEG = 45.0;

    // The indexer only feeds once the robot's heading is within this many degrees of the shooting heading.
    // Every measured scoring shot was within 5 deg of the computed heading.
    public static final double HEADING_TOLERANCE_DEG = 3.0;

    // Aiming turn controller (X/B held): turn power = HEADING_KP * error - HEADING_KD * turn rate, plus HEADING_KS
    // in the direction of the error while the error is bigger than HEADING_KS_DEADBAND_DEG, clamped to [-1, 1].
    // P alone (the old controller) asked for too little power near the target to overcome friction, so the robot
    // stalled a few degrees short of HEADING_TOLERANCE_DEG and never fed. Tune on the robot with the TeleOp log:
    //   stops short of the target -> raise HEADING_KS;  wobbles back and forth -> raise HEADING_KD or lower HEADING_KP;
    //   turns too slowly from far away -> raise HEADING_KP.
    // Turn power per radian of heading error
    public static final double HEADING_KP = 1.2;
    // Turn power per radian/second of turning, subtracted to brake the turn as it nears the target
    public static final double HEADING_KD = 0.06;
    // Minimum turn power that overcomes friction, added in the direction of the error
    public static final double HEADING_KS = 0.07;
    // Below this much error, HEADING_KS is left off so the robot doesn't jitter around the target. Must be smaller
    // than HEADING_TOLERANCE_DEG so the robot still gets inside the tolerance.
    public static final double HEADING_KS_DEADBAND_DEG = 1.0;

    /**
     * Not meant to be instantiated; this class only holds constants.
     */
    private RobotConstants() {
    }
}
