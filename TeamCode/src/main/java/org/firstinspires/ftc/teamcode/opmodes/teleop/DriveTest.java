package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Field-centric TeleOp driven by Pedro Pathing. The robot's pose from the Pinpoint computer
 * is sent to telemetry, along with the shooter wheel's speed. Pressing AUTO_SHOOT_BUTTON runs the
 * auto-shoot routine (an Ivy command, see Robot.startAutoShoot()), so this OpMode drives the Ivy scheduler.
 */
@TeleOp(name = "DriveTest")
public class DriveTest extends OpMode {
    // Auto-shoot routine for this test. Pedro coordinates: x and y in inches, heading in degrees
    // (counterclockwise is positive), measured from the start / zero pose.
    // Legs, in order: drive to AUTO_SHOOT_WALL_CLEARANCE_Y_INCHES, drive to the shooting x, turn to the
    // shooting heading, drive to the shooting y. Then shoot for AUTO_SHOOT_DURATION_SECONDS.
    private static final double SHOOTING_POSE_X_INCHES = 75.0;
    private static final double SHOOTING_POSE_Y_INCHES = 2.0;
    private static final double SHOOTING_POSE_HEADING_DEGREES = 268.2962;
    // Far enough off the wall that the robot doesn't catch it while driving along x
    private static final double AUTO_SHOOT_WALL_CLEARANCE_Y_INCHES = 21.0;
    private static final double AUTO_SHOOT_DURATION_SECONDS = 8.0;
    private static final double AUTO_SHOOT_TARGET_RPM = 2400.0;

    private Robot robot;

    /**
     * Runs once when INIT is pressed. Clears any commands left over from a previous OpMode, creates the
     * robot, which initializes all hardware, gives it this test's auto-shoot numbers, and sets the pose to the start pose (bottom right corner,
     * facing forward), so field-centric "forward" is whichever way the robot faces at INIT.
     */
    @Override
    public void init() {
        // The Ivy scheduler is static and outlives OpModes, so clear anything a previous OpMode left behind
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        robot.configureAutoShoot(
                new Pose(SHOOTING_POSE_X_INCHES, SHOOTING_POSE_Y_INCHES, Math.toRadians(SHOOTING_POSE_HEADING_DEGREES)),
                AUTO_SHOOT_WALL_CLEARANCE_Y_INCHES,
                AUTO_SHOOT_DURATION_SECONDS,
                AUTO_SHOOT_TARGET_RPM
        );
        // The Pinpoint keeps its pose from the previous OpMode; without this, a robot that ended an autonomous
        // facing 180 deg would drive with forward/back and left/right reversed, and the shooting pose would be off
        robot.drivetrain.setPose(Robot.startPose());
        robot.update(); // apply the start pose before the OpMode starts
    }

    /**
     * Runs repeatedly after START. Applies the driver controls, advances the auto-shoot routine if it's
     * running, updates the robot, then reports the robot pose to telemetry.
     */
    @Override
    public void loop() {
        robot.applyDriverControls(gamepad1);
        // One non-blocking pass over every scheduled command; this is what advances the auto-shoot routine
        Scheduler.execute();
        robot.update();

        Pose robotPose = robot.drivetrain.getPose(); // position from the Pinpoint; inches for x/y, radians for heading
        telemetry.addData("Robot X (in)", robotPose.x());
        telemetry.addData("Robot Y (in)", robotPose.y());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotPose.heading())); // radians -> degrees for reading
        telemetry.addData("Heading Locked", robot.drivetrain.isHeadingLocked());
        telemetry.addData("Robot State", robot.getSuperState().label);
        telemetry.addData("Auto-shoot running", robot.isAutoShooting());
        telemetry.addData("Shooter RPM", robot.shooter.getShooterRpm());
        telemetry.addData("Shooter Target RPM", robot.shooter.getTargetRpm()); // adjusted live with the bumpers
        telemetry.addData("Shooter Peak RPM", robot.shooter.getPeakShooterRpm());
        telemetry.addData("Shooter Encoder (ticks/s)", robot.shooter.getShooterTicksPerSecond());
        telemetry.update();
    }
}
