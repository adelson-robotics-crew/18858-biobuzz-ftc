package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Odometry test: spins the robot 360 deg counterclockwise in place, pauses, then spins it 360 deg
 * clockwise back, as one sequential Ivy command. If the odometry is good, the robot physically ends
 * where it started and telemetry shows little position drift and a total rotation near 0.
 */
@Autonomous(name = "Rotate360WithPauses")
public class Rotate360WithPauses extends OpMode {
    private Robot robot;

    // The whole routine, built in start(); kept so telemetry can report whether it is still running
    private Command rotateRoutine;

    // How long the robot sits still between the counterclockwise and clockwise spins
    private static final double PAUSE_BETWEEN_SPINS_MILLISECONDS = 1000;

    // Creates poses from (x, y, heading) with the heading given in degrees
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Every pose is the same spot; only the heading changes. Heading increases counterclockwise.
    // Holding a pose always turns the short way to it, and 360 deg is the same heading as 0 deg,
    // so each full spin is made of four 90 deg steps, which leaves no doubt about which way to turn.
    private final Pose startPose = poseFactory.of(72, 72, 0);
    private final Pose heading90Pose = poseFactory.of(72, 72, 90);
    private final Pose heading180Pose = poseFactory.of(72, 72, 180);
    private final Pose heading270Pose = poseFactory.of(72, 72, 270);

    // Heading from the previous loop, used to add up how far the robot has turned in total
    private double previousHeadingRadians;

    // Total rotation since START, counterclockwise positive. Unlike the pose heading it does not
    // wrap at 360 deg, so it reads about 360 after the first spin and about 0 after the second.
    private double totalRotationDegrees = 0.0;

    /**
     * Runs once when INIT is pressed. Clears any commands left over from a previous OpMode,
     * creates the robot, and tells it where the robot starts.
     */
    @Override
    public void init() {
        // The Ivy scheduler is static and outlives OpModes, so clear anything a previous OpMode left behind
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPose(startPose);
        robot.update(); // apply the starting pose before the OpMode starts
    }

    /**
     * Runs once when START is pressed. Builds the spin routine and schedules it.
     */
    @Override
    public void start() {
        previousHeadingRadians = robot.drivetrain.getPose().heading();
        totalRotationDegrees = 0.0;

        rotateRoutine = sequential(
                // 360 deg counterclockwise: 0 -> 90 -> 180 -> 270 -> 0
                robot.drivetrain.holdPoseCommand(heading90Pose),
                robot.drivetrain.holdPoseCommand(heading180Pose),
                robot.drivetrain.holdPoseCommand(heading270Pose),
                robot.drivetrain.holdPoseCommand(startPose),
                waitMs(PAUSE_BETWEEN_SPINS_MILLISECONDS), // the drivetrain keeps holding startPose during the pause
                // 360 deg clockwise: 0 -> 270 -> 180 -> 90 -> 0
                robot.drivetrain.holdPoseCommand(heading270Pose),
                robot.drivetrain.holdPoseCommand(heading180Pose),
                robot.drivetrain.holdPoseCommand(heading90Pose),
                robot.drivetrain.holdPoseCommand(startPose)
        );
        Scheduler.schedule(rotateRoutine);
    }

    /**
     * Runs repeatedly after START. Advances the scheduled commands by one step, updates the robot,
     * then reports how far the robot has drifted and turned.
     */
    @Override
    public void loop() {
        // One non-blocking pass over every scheduled command; this is what advances the routine
        Scheduler.execute();
        robot.update(); // must be called every loop for the robot to move

        Pose robotPose = robot.drivetrain.getPose(); // inches for x/y, radians for heading

        // Angle.error(from, to) is (to - from) wrapped into [-180, 180) deg (as radians), so crossing
        // 360 -> 0 counts as a small step instead of a jump, which lets the total keep counting past 360
        totalRotationDegrees += Math.toDegrees(Angle.error(previousHeadingRadians, robotPose.heading()));
        previousHeadingRadians = robotPose.heading();

        telemetry.addData("Routine running", Scheduler.isScheduled(rotateRoutine));
        telemetry.addData("Total rotation (deg, CCW +)", totalRotationDegrees);
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Position drift from start (in)", robotPose.distance(startPose));
        telemetry.addData("X (in)", robotPose.x());
        telemetry.addData("Y (in)", robotPose.y());
        telemetry.update();
    }
}
