package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.logging.CsvLog;

/**
 * The match autonomous: starts with 4 balls, shifts over slightly to the shooting spot, shoots for a fixed time,
 * then drives to the parking square and turns so the intake faces the field for TeleOp. One sequential Ivy
 * command; each drive leg finishes only once the robot is at its target pose (see Drivetrain's followPathCommand()
 * and holdPoseCommand()), and the Ivy scheduler runs the sequence from loop().
 * Every loop is written to a CSV log on the Robot Controller (see CsvLog) so a run can be analyzed afterward.
 */
@Autonomous(name = "Match Auto")
public class MatchAuto extends OpMode {
    private Robot robot;

    // One row per loop, written to /sdcard/FIRST/logs/ on the Robot Controller
    private CsvLog matchLog;
    // Time since START, for the log's time column; and the time of the previous loop, for measuring loop length
    private final ElapsedTime timeSinceStart = new ElapsedTime();
    private double previousLoopSeconds = 0.0;
    // Which step of the routine is running and the pose it's heading for, set by markStep() as each step begins.
    // Logged every loop so the data can be lined up with the routine
    private String currentStepName = "init";
    private Pose currentStepTargetPose = null;

    // The whole routine, built in start(); kept so telemetry can report whether it is still running
    private Command matchRoutine;

    // Creates poses from (x, y, heading) with the heading given in degrees
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Pedro coordinates, inches. Heading 90 deg = front of the robot faces +y (toward the wall behind the start),
    // so the shooter (out the back) faces -y toward the HIVE. The robot keeps 90 deg for every drive leg and only
    // turns at the very end. The start heading must match how the robot is really placed: telling the localizer
    // 270 here (an earlier version) mirrors every move, so the robot slid right and backed into the wall.
    private final Pose startPose = poseFactory.of(62.22, 132.56, 90);
    private final Pose shootingPose = poseFactory.of(60, 132.56, 90);           // slide over in x only (2.22 in)
    private final Pose leaveShootingPose = poseFactory.of(57.87, 107.03, 90);   // drive away from the wall in y only
    private final Pose parkedTurnedPose = poseFactory.of(15, 120, 180);         // park, turning 90 deg counterclockwise on the way

    /**
     * Builds a straight path whose heading turns evenly from pathStartPose's heading to pathEndPose's heading
     * as the robot moves along it (constant the whole way if the two headings match).
     *
     * @param pathStartPose where the path starts
     * @param pathEndPose   where the path ends
     * @return the straight path from pathStartPose to pathEndPose
     */
    private Path straightPath(Pose pathStartPose, Pose pathEndPose) {
        return line(pathStartPose, pathEndPose).linear(pathStartPose, pathEndPose);
    }

    /**
     * Builds a command that records which step of the routine is starting, for the log. Put one in front of
     * each step in the sequence.
     *
     * @param stepName   the name written to the log's step column
     * @param targetPose the pose the step is driving to, or null if it doesn't drive
     * @return an instant command that records the step
     */
    private Command markStep(String stepName, Pose targetPose) {
        return instant(() -> {
            currentStepName = stepName;
            currentStepTargetPose = targetPose;
        });
    }

    /**
     * Builds the shooting step: spins the flywheel up to RobotConstants.AUTO_SHOOTER_TARGET_RPM, waits for it to
     * reach speed (within RobotConstants.INDEXER_START_FEED_RPM_TOLERANCE, the same check TeleOp uses), shoots for
     * RobotConstants.AUTO_SHOOT_SECONDS, then stops the shooter. The indexer feeds by itself whenever the wheel is
     * at speed, so no ball is fed into a slow wheel. The drivetrain keeps holding the shooting pose throughout.
     *
     * @return a command that finishes once shooting is done and the shooter is stopped
     */
    private Command shootAllBallsCommand() {
        return sequential(
                instant(() -> {
                    robot.shooter.setTargetRpm(RobotConstants.AUTO_SHOOTER_TARGET_RPM);
                    robot.shooter.setFeedAllowed(true); // no aiming in auto; feed whenever the wheel is at speed
                    robot.requestShoot();
                }),
                // Whichever finishes first: the wheel reaches speed, or the spin-up timeout runs out
                race(
                        waitUntil(() -> robot.shooter.isAtSpeed()),
                        waitMs(RobotConstants.AUTO_SPIN_UP_TIMEOUT_SECONDS * 1000.0)
                ),
                waitMs(RobotConstants.AUTO_SHOOT_SECONDS * 1000.0),
                instant(() -> robot.stopShoot())
        );
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

        matchLog = new CsvLog("MatchAuto",
                "timeSec", "loopMs", "step",
                "x", "y", "headingDeg",
                "targetX", "targetY", "targetHeadingDeg",
                "velocityX", "velocityY", "turnRateDegPerSec",
                "followerMode", "followerBusy", "pathProgress",
                "pathPointX", "pathPointY", "pathPointHeadingDeg",
                "shooterRpm", "shooterTargetRpm", "shooterAtSpeed",
                "pedroDebug");
    }

    /**
     * Runs once when START is pressed. Builds the match routine and schedules it.
     */
    @Override
    public void start() {
        matchRoutine = sequential(
                markStep("slideToShoot", shootingPose),
                robot.drivetrain.followPathCommand(straightPath(startPose, shootingPose)),
                markStep("shoot", null),
                shootAllBallsCommand(),
                markStep("leaveWall", leaveShootingPose),
                robot.drivetrain.followPathCommand(straightPath(shootingPose, leaveShootingPose)),
                markStep("park", parkedTurnedPose),
                robot.drivetrain.followPathCommand(straightPath(leaveShootingPose, parkedTurnedPose)),
                markStep("done", null)
        );
        Scheduler.schedule(matchRoutine);
        timeSinceStart.reset();
        previousLoopSeconds = 0.0;
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
        telemetry.addData("Routine running", Scheduler.isScheduled(matchRoutine));
        telemetry.addData("X (in)", robotPose.x());
        telemetry.addData("Y (in)", robotPose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Follower busy", robot.drivetrain.isBusy());
        telemetry.addData("Shooter RPM", robot.shooter.getShooterRpm());
        telemetry.addData("Shooter target RPM", robot.shooter.getTargetRpm());
        telemetry.addData("Shooter at speed", robot.shooter.isAtSpeed());
        telemetry.addData("Step", currentStepName);
        telemetry.addData("Log file", matchLog.getFilePath());
        if (matchLog.getErrorMessage() != null) {
            telemetry.addData("Log error", matchLog.getErrorMessage());
        }
        telemetry.update();

        logThisLoop(robotPose);
    }

    /**
     * Writes one row to the match log describing this loop. Call after robot.update(), so every value is from
     * this loop. Headings are logged in degrees; values that don't apply right now (no target, not following a
     * path) are left blank.
     *
     * @param robotPose the robot's pose this loop
     */
    private void logThisLoop(Pose robotPose) {
        double nowSeconds = timeSinceStart.seconds();
        double loopMilliseconds = (nowSeconds - previousLoopSeconds) * 1000.0;
        previousLoopSeconds = nowSeconds;

        Velocity robotVelocity = robot.drivetrain.getVelocity();
        Pose closestPathPose = robot.drivetrain.getClosestPathPose(); // null while not following a path
        double pathProgress = robot.drivetrain.getPathProgress();     // NaN while not following a path

        matchLog.addRow(
                nowSeconds, loopMilliseconds, currentStepName,
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()),
                currentStepTargetPose == null ? "" : currentStepTargetPose.x(),
                currentStepTargetPose == null ? "" : currentStepTargetPose.y(),
                currentStepTargetPose == null ? "" : Math.toDegrees(currentStepTargetPose.heading()),
                robotVelocity.vx, robotVelocity.vy, Math.toDegrees(robotVelocity.omega),
                robot.drivetrain.getFollowerModeName(), robot.drivetrain.isBusy(),
                Double.isNaN(pathProgress) ? "" : pathProgress,
                closestPathPose == null ? "" : closestPathPose.x(),
                closestPathPose == null ? "" : closestPathPose.y(),
                closestPathPose == null ? "" : Math.toDegrees(closestPathPose.heading()),
                robot.shooter.getShooterRpm(), robot.shooter.getTargetRpm(), robot.shooter.isAtSpeed(),
                robot.drivetrain.getFollowerDebugText());
    }

    /**
     * Runs once when STOP is pressed. Stops the shooter in case the match ended mid-shot.
     */
    @Override
    public void stop() {
        robot.stopShoot();
        robot.update(); // apply the stop right away
        matchLog.close(); // writes out the rows still buffered
    }
}
