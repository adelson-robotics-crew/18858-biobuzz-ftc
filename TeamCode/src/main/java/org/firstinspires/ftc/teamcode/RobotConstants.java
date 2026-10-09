package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/**
 * Definitions only, no logic: the constants that more than one class reads. Two sections: the gamepad button
 * mapping (read by Robot and the TeleOp), and the few tunables shared between classes.
 * A constant that only one class uses lives at the top of that class instead (hardware names, tuning gains, and
 * so on), so it sits next to the code it tunes. Pedro's own tuned values (PIDs, pod offsets, drivetrain motor
 * names, etc.) stay in subsystems/drivetrain/pedro/Constants.java.
 * All positions are Pedro field coordinates: x and y in inches, heading in degrees (counterclockwise is positive).
 */
public final class RobotConstants {

    // =====================================================================================
    // GAMEPAD BUTTON MAPPING
    // Every control on the driver gamepad (gamepad1), kept together so all controls can be seen and remapped in
    // one place. Each constant reads one gamepad field directly, so to remap a control, change its line here.
    // Nothing else needs to change.
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
    // Outtake: runs the intake servos and motor backwards to push balls out. If it's held with INTAKE_BUTTON,
    // intake wins. Shares the left bumper with SHOOTER_RPM_DOWN_BUTTON, which only works while
    // SHOOTER_RPM_ADJUST_ENABLED is on (off for matches).
    public static final Predicate<Gamepad> OUTTAKE_BUTTON = gamepad -> gamepad.left_bumper;
    // Manual shot: spins the flywheel at the fixed SHOOTER_TARGET_RPM (not the shot table, so it still works if the
    // robot's position is off) and the indexer feeds automatically whenever the wheel is up to speed. The driver
    // drives to the usual spot and shoots from there.
    public static final Predicate<Gamepad> SHOOT_BUTTON = gamepad -> gamepad.a;

    // Shooter RPM tuning on the fly. Each press (not hold) bumps the shooter target RPM by Robot's
    // SHOOTER_RPM_ADJUST_STEP. Off for matches, so a bumper pressed by accident can't change the shooting speed.
    // Turn on to tune RPM.
    public static final boolean SHOOTER_RPM_ADJUST_ENABLED = false;
    public static final Predicate<Gamepad> SHOOTER_RPM_UP_BUTTON = gamepad -> gamepad.right_bumper;
    public static final Predicate<Gamepad> SHOOTER_RPM_DOWN_BUTTON = gamepad -> gamepad.left_bumper;

    // Aim and shoot (hold): the robot turns to face the shooter at a HIVE cell while the left stick still drives,
    // the shooter spins at the shot table's RPM for the current distance, and the indexer feeds only while the
    // position is valid (distance and angle), the heading is within Robot's HEADING_TOLERANCE_DEG, and the wheel is
    // at speed. The turn stick is ignored while held. Release to go back to normal driving; the shooter stops.
    // One button for both cells: on press it picks the alliance's cell on the robot's half of the field (the robot's
    // y vs. the field's horizontal center line) and keeps that cell until released (see Robot.chooseShotTarget()).
    public static final Predicate<Gamepad> AIM_AND_SHOOT_BUTTON =
            gamepad -> gamepad.right_trigger > TRIGGER_PRESSED_THRESHOLD;

    // Emergency heading reset (one press): with the robot facing straight away from the driver (the way the
    // forward stick drives), sets the heading to 0 deg from the red side or 180 deg from the blue side. In Match
    // TeleOp x and y are left alone; it's only for when the heading has drifted or the robot got spun. In DriveTest
    // it resets the whole pose to (0, 0, 0 deg), so put the robot back in the practice corner, facing away from the
    // driver, before pressing it.
    // "back" is the small button on the left side above the mode button (labeled Share on PlayStation controllers).
    public static final Predicate<Gamepad> RESET_HEADING_BUTTON = gamepad -> gamepad.back;

    // =====================================================================================
    // SHARED TUNABLES
    // Each of these is read by more than one class.
    // =====================================================================================

    // Shooter speed (Robot and Shooter)

    // Speed the shooter wheel is held at for a manual shot (SHOOT_BUTTON), in RPM of the motor's output shaft.
    // Aim-and-shoot uses the shot table instead. With SHOOTER_RPM_ADJUST_ENABLED on, the bumpers bump the manual
    // speed up/down during TeleOp; once a good value is found from telemetry, copy it here.
    // Held with setVelocity(), so it stays the same as the battery drains. Keep it comfortably below
    // the motor's free speed (6000 RPM) so the controller has headroom on a low battery.
    public static final double SHOOTER_TARGET_RPM = 2425.0;

    // The adjusted target RPM is kept within these limits. The max is the motor's free speed; above it
    // the wheel can never reach the target, so the indexer would never feed.
    public static final double SHOOTER_MIN_TARGET_RPM = 0.0;
    public static final double SHOOTER_MAX_TARGET_RPM = 6000.0;

    // Field geometry (Alliance and ShotTarget)

    // The point the field is symmetric about. The four cells are mirror images of each other around it:
    // blue lower (81.45, 56.06), blue upper (81.45, 85.44), red upper (60.05, 85.44), red lower (60.05, 56.06).
    public static final double FIELD_SYMMETRY_CENTER_X_INCHES = 70.75;
    public static final double FIELD_SYMMETRY_CENTER_Y_INCHES = 70.75;

    // Shot validity (ShotSolver and ShotTarget)

    // A position is only valid within this many degrees either side of straight in front of the cell
    // (see ShotTarget.frontDirectionDegrees).
    public static final double MAX_ANGLE_OFF_CENTER_DEG = 45.0;

    /**
     * Not meant to be instantiated; this class only holds constants.
     */
    private RobotConstants() {
    }
}
