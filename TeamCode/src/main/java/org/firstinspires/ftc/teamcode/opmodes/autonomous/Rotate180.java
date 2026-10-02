package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Turns the robot 180 deg in place and holds it there. A single step needs no sequencing,
 * so this tells the drivetrain to hold the turned pose directly instead of using Ivy commands.
 */
@Autonomous(name = "Rotate180")
public class Rotate180 extends OpMode {
    private Robot robot;

    // Creates poses from (x, y, heading) with the heading given in degrees
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(72, 72, 0);
    private final Pose turnedPose = poseFactory.of(72, 72, 180); // same spot, turned 180 deg

    /**
     * Runs once when INIT is pressed. Creates the robot and tells it where the robot starts.
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPose(startPose);
        robot.update(); // apply the starting pose before the OpMode starts
    }

    /**
     * Runs once when START is pressed. Tells the drivetrain to turn to 180 deg and hold there.
     */
    @Override
    public void start() {
        robot.drivetrain.holdPose(turnedPose);
    }

    /**
     * Runs repeatedly after START. Updates the robot and reports its pose.
     */
    @Override
    public void loop() {
        robot.update(); // must be called every loop for the robot to move

        Pose robotPose = robot.drivetrain.getPose(); // inches for x/y, radians for heading
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.addData("X (in)", robotPose.x());
        telemetry.addData("Y (in)", robotPose.y());
        telemetry.update();
    }
}
