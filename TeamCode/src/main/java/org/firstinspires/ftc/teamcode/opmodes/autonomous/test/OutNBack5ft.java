package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Drives 5 ft out, spins 180 deg, drives 5 ft back, and spins 180 deg again, as one sequential
 * Ivy command. Each leg finishes only once the robot is at its target pose (see Drivetrain's
 * followPathCommand() and holdPoseCommand()), and the Ivy scheduler runs the sequence from loop().
 */
@Autonomous(name = "OutNBack5ft")
public class OutNBack5ft extends OpMode {
    private Robot robot;

    // The whole routine, built in start(); kept so telemetry can report whether it is still running
    private Command outAndBackRoutine;

    // Creates poses from (x, y, heading) with the heading given in degrees
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches (1 ft = 12 in).
    // Out: 5 ft along +x facing 0 deg. Spin 180 deg in place. Back: 5 ft along -x facing 180 deg.
    // Spin 180 deg in place again so the robot ends facing the way it started and the routine can be re-run.
    private final Pose startPose = poseFactory.of(12, 48, 0);
    private final Pose outEndPose = poseFactory.of(72, 48, 0);       // startPose + 60 in in x
    private final Pose backStartPose = poseFactory.of(72, 48, 180);  // same spot as outEndPose, spun 180 deg
    private final Pose backEndPose = poseFactory.of(12, 48, 180);    // backStartPose - 60 in in x
    private final Pose finalPose = poseFactory.of(12, 48, 0);        // same spot as backEndPose, spun back to 0 deg

    /**
     * Builds the straight path that drives the robot 5 ft forward, keeping its heading at 0 degrees.
     *
     * @return the path from the start pose to the end of the outbound leg
     */
    private Path outPath() {
        return line(startPose, outEndPose).linear(startPose, outEndPose);
    }

    /**
     * Builds the straight path that drives the robot 5 ft back, keeping its heading at 180 degrees.
     *
     * @return the path from the start of the return leg to its end
     */
    private Path backPath() {
        return line(backStartPose, backEndPose).linear(backStartPose, backEndPose);
    }

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
     * Runs once when START is pressed. Builds the out-and-back routine and schedules it.
     */
    @Override
    public void start() {
        outAndBackRoutine = sequential(
                robot.drivetrain.followPathCommand(outPath()),
                robot.drivetrain.holdPoseCommand(backStartPose), // rotate 180 deg without moving
                robot.drivetrain.followPathCommand(backPath()),
                robot.drivetrain.holdPoseCommand(finalPose)      // rotate back to face the way it started
        );
        Scheduler.schedule(outAndBackRoutine);
    }

    /**
     * Runs repeatedly after START. Advances the scheduled commands by one step, then updates the robot.
     */
    @Override
    public void loop() {
        // One non-blocking pass over every scheduled command; this is what advances the routine
        Scheduler.execute();
        robot.update(); // must be called every loop for the robot to move

        Pose robotPose = robot.drivetrain.getPose(); // inches for x/y, radians for heading
        telemetry.addData("Routine running", Scheduler.isScheduled(outAndBackRoutine));
        telemetry.addData("X (in)", robotPose.x());
        telemetry.addData("Y (in)", robotPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Follower busy", robot.drivetrain.isBusy());
        telemetry.update();
    }
}
