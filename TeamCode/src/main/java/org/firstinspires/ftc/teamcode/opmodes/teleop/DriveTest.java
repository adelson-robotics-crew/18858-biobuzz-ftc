package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;

/**
 * Field-centric TeleOp driven by Pedro Pathing. The robot's pose from the Pinpoint computer
 * is sent to telemetry, along with the shooter wheel's speed. Holding AIM_SHOOT_BUTTON aims and shoots
 * (see Robot.applyDriverControls()), and adds the shot's numbers to telemetry while held.
 */
@TeleOp(name = "DriveTest")
public class DriveTest extends OpMode {
    private Robot robot;

    /**
     * Runs once when INIT is pressed. Creates the robot, which initializes all hardware, and sets the pose to
     * the start pose (bottom right corner, facing forward), so field-centric "forward" is whichever way the
     * robot faces at INIT.
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        // The Pinpoint keeps its pose from the previous OpMode; without this, a robot that ended an autonomous
        // facing 180 deg would drive with forward/back and left/right reversed, and aiming would be off
        robot.drivetrain.setPose(Robot.startPose());
        robot.update(); // apply the start pose before the OpMode starts
    }

    /**
     * Runs repeatedly after START. Applies the driver controls, updates the robot, then reports the robot
     * pose, shooter, and (while aiming) shot numbers to telemetry.
     */
    @Override
    public void loop() {
        robot.applyDriverControls(gamepad1);
        robot.update();

        Pose robotPose = robot.drivetrain.getPose(); // position from the Pinpoint; inches for x/y, radians for heading
        telemetry.addData("Robot X (in)", robotPose.x());
        telemetry.addData("Robot Y (in)", robotPose.y());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotPose.heading())); // radians -> degrees for reading
        telemetry.addData("Heading Locked", robot.drivetrain.isHeadingLocked());
        telemetry.addData("Robot State", robot.getSuperState().label);

        ShotSolution shotSolution = robot.getShotSolution(); // null unless AIM_SHOOT_BUTTON is held
        if (shotSolution != null) {
            telemetry.addData("Shot READY", robot.isReadyToShoot());
            telemetry.addData("Shot Position", shotSolution.validity.label);
            telemetry.addData("Shot Distance (in)", "%.1f", shotSolution.distanceInches);
            telemetry.addData("Shot Angle Off Center (deg)", "%.1f", shotSolution.angleOffCenterDegrees);
            telemetry.addData("Shot Target Heading (deg)", "%.1f", Math.toDegrees(shotSolution.targetHeadingRadians));
            telemetry.addData("Shot Heading Error (deg)", "%.1f", robot.getAimHeadingErrorDeg());
            telemetry.addData("Shot Target RPM", "%.0f", shotSolution.targetRpm);
            telemetry.addData("Shot Actual RPM", "%.0f", robot.shooter.getShooterRpm());
        }

        telemetry.addData("Shooter RPM", robot.shooter.getShooterRpm());
        telemetry.addData("Shooter Target RPM", robot.shooter.getTargetRpm()); // adjusted live with the bumpers
        telemetry.addData("Shooter At Speed", robot.shooter.isAtSpeed());
        telemetry.addData("Shooter Peak RPM", robot.shooter.getPeakShooterRpm());
        telemetry.addData("Shooter Encoder (ticks/s)", robot.shooter.getShooterTicksPerSecond());
        telemetry.update();
    }
}
