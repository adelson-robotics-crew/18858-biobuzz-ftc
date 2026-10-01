package org.firstinspires.ftc.teamcode.opmodes.autonomous;

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
 * Drives a 5 ft square, turning 90 deg to the right (clockwise) in place at every corner, as one
 * sequential Ivy command. Each side and each turn finishes only once the robot is at its target
 * pose (see Drivetrain's followPathCommand() and holdPoseCommand()). After the last side the robot
 * turns back to its starting heading, so it ends where it started and the routine can be re-run.
 */
@Autonomous(name = "FiveFootSquare")
public class FiveFootSquare extends OpMode {
    private Robot robot;

    // The whole routine, built in start(); kept so telemetry can report whether it is still running
    private Command squareRoutine;

    // Creates poses from (x, y, heading) with the heading given in degrees
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates are in inches (1 ft = 12 in), and heading increases counterclockwise,
    // so each right-hand turn takes the heading down by 90 deg: 0 -> 270 -> 180 -> 90 -> 0.
    // Corners, in driving order: one (12, 72) -> two (72, 72) -> three (72, 12) -> four (12, 12) -> one.
    private final Pose cornerOneHeading0 = poseFactory.of(12, 72, 0);      // start: facing +x
    private final Pose cornerTwoHeading0 = poseFactory.of(72, 72, 0);      // cornerOne + 60 in in x
    private final Pose cornerTwoHeading270 = poseFactory.of(72, 72, 270);  // same spot, turned right to face -y
    private final Pose cornerThreeHeading270 = poseFactory.of(72, 12, 270); // cornerTwo - 60 in in y
    private final Pose cornerThreeHeading180 = poseFactory.of(72, 12, 180); // same spot, turned right to face -x
    private final Pose cornerFourHeading180 = poseFactory.of(12, 12, 180); // cornerThree - 60 in in x
    private final Pose cornerFourHeading90 = poseFactory.of(12, 12, 90);   // same spot, turned right to face +y
    private final Pose cornerOneHeading90 = poseFactory.of(12, 72, 90);    // cornerFour + 60 in in y
    // cornerOneHeading0 again is the final pose: same spot, turned right to face +x like at the start

    /**
     * Builds a straight side of the square that keeps the robot's heading constant the whole way.
     *
     * @param sideStartPose where the side starts; must have the same heading as sideEndPose
     * @param sideEndPose   where the side ends
     * @return the straight path from sideStartPose to sideEndPose
     */
    private Path straightSide(Pose sideStartPose, Pose sideEndPose) {
        return line(sideStartPose, sideEndPose).linear(sideStartPose, sideEndPose);
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
        robot.drivetrain.setPose(cornerOneHeading0);
        robot.update(); // apply the starting pose before the OpMode starts
    }

    /**
     * Runs once when START is pressed. Builds the square routine and schedules it.
     */
    @Override
    public void start() {
        squareRoutine = sequential(
                robot.drivetrain.followPathCommand(straightSide(cornerOneHeading0, cornerTwoHeading0)),
                robot.drivetrain.holdPoseCommand(cornerTwoHeading270),   // turn right 90 deg in place
                robot.drivetrain.followPathCommand(straightSide(cornerTwoHeading270, cornerThreeHeading270)),
                robot.drivetrain.holdPoseCommand(cornerThreeHeading180), // turn right 90 deg in place
                robot.drivetrain.followPathCommand(straightSide(cornerThreeHeading180, cornerFourHeading180)),
                robot.drivetrain.holdPoseCommand(cornerFourHeading90),   // turn right 90 deg in place
                robot.drivetrain.followPathCommand(straightSide(cornerFourHeading90, cornerOneHeading90)),
                robot.drivetrain.holdPoseCommand(cornerOneHeading0)      // turn right 90 deg back to the starting heading
        );
        Scheduler.schedule(squareRoutine);
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
        telemetry.addData("Routine running", Scheduler.isScheduled(squareRoutine));
        telemetry.addData("X (in)", robotPose.x());
        telemetry.addData("Y (in)", robotPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Follower busy", robot.drivetrain.isBusy());
        telemetry.update();
    }
}
