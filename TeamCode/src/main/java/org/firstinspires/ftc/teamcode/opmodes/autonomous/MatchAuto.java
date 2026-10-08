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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.logging.CsvLog;
import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;

/**
 * The match autonomous: starts with 4 balls and shoots them from the starting pose, then pulls away from the wall
 * and strafes into the parking square. The heading never changes (90 deg on red, 270 deg on blue). One sequential Ivy
 * command; each drive leg finishes only once the robot is at its target pose (see Drivetrain's
 * followPathCommand()), and the Ivy scheduler runs the sequence from loop().
 * Every loop is written to a CSV log on the Robot Controller (see CsvLog) so a run can be analyzed afterward.
 * Not listed on the Driver Station itself: RedMatchAuto and BlueMatchAuto choose the alliance. The routine is
 * written once with red coordinates, and blue runs it rotated 180 deg about the field center. Run Match TeleOp
 * right after; it picks up this auto's final pose and alliance.
 */
public abstract class MatchAuto extends OpMode {
    private Robot robot;

    // One row per loop, written to /sdcard/logs/ on the Robot Controller
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

    // Pedro coordinates, inches. On red, heading 90 deg = front of the robot faces +y (toward the wall behind the
    // start), so the shooter (out the back) faces -y toward the red upper cell. The heading never changes.
    // The start heading must match how the robot is really placed: telling the localizer the opposite heading
    // turns every move around, so the robot slid the wrong way and backed into the wall.
    // These are the RED side's poses. Blue uses the same ones rotated 180 deg about the field center
    // (see Alliance.fromRedSide()): blue starts at the bottom of the field facing 270 deg, and every move is turned
    // around to match. Only red's are written down; init() picks this alliance's version.
    private final Pose redStartPose = poseFactory.of(58.664, 133.403, 90);      // also where the robot shoots from
    private final Pose redLeaveWallPose = poseFactory.of(59.207, 115.477, 90);  // pull away from the wall (about 18 in)
    private final Pose redParkingPose = poseFactory.of(12.7, 117.92, 90);       // strafe into the parking square

    // How many times init_loop() had to set the start pose again because the odometry didn't read it, shown on
    // telemetry (and logged) to tell whether the Pinpoint is losing the start pose
    private int startPoseResets = 0;

    // This alliance's versions of the poses above, set in init()
    private Pose startPose;
    private Pose leaveWallPose;
    private Pose parkingPose;

    /**
     * Which alliance this auto runs for. Picking the red or blue auto is how the robot knows its side: the auto
     * saves it (Robot.saveAlliance()) and the TeleOp that follows aims at that alliance's hive.
     *
     * @return the alliance
     */
    protected abstract Alliance alliance();

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
     * at speed, so no ball is fed into a slow wheel. The drivetrain is stopped throughout.
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
        startPose = alliance().fromRedSide(redStartPose);
        leaveWallPose = alliance().fromRedSide(redLeaveWallPose);
        parkingPose = alliance().fromRedSide(redParkingPose);
        robot = new Robot(hardwareMap);
        Robot.saveAlliance(alliance()); // the TeleOp that follows aims at this alliance's hive
        robot.drivetrain.setPose(startPose);
        robot.update(); // apply the starting pose before the OpMode starts

        matchLog = new CsvLog("MatchAuto" + alliance().label,
                "timeSec", "loopMs", "step",
                "x", "y", "headingDeg",
                "targetX", "targetY", "targetHeadingDeg",
                "velocityX", "velocityY", "turnRateDegPerSec",
                "followerMode", "followerBusy", "pathProgress",
                "pathPointX", "pathPointY", "pathPointHeadingDeg",
                "shooterRpm", "shooterTargetRpm", "shooterAtSpeed",
                "indexerFeeding", "indexerAngleDeg", "indexerOffRestDeg",
                "pedroDebug");
    }

    /**
     * Runs repeatedly between INIT and START. Keeps the odometry updating and makes sure it reads the starting pose:
     * creating the robot recalibrates the Pinpoint's gyro, which takes about a quarter second, and a start pose set
     * while that's still running can be lost. So if the odometry doesn't read the start pose, it's set again (the
     * robot is sitting at the start spot during INIT). If it still won't take, the Driver Station says so.
     */
    @Override
    public void init_loop() {
        robot.update();
        if (!robot.drivetrain.isAtPose(startPose)) {
            robot.drivetrain.setPose(startPose);
            startPoseResets++;
        }
        Pose robotPose = robot.drivetrain.getPose();
        if (robot.drivetrain.isAtPose(startPose)) {
            telemetry.addData("Odometry", "OK: reads the %s start pose", alliance().label);
        } else {
            telemetry.addData("ODOMETRY ERROR", "Doesn't read the start pose even after setting it again. Check the "
                    + "Pinpoint and pod cables, then power-cycle the robot. Don't run the auto like this.");
        }
        telemetry.addData("Start pose set again (times)", startPoseResets);
        telemetry.addData("Start pose (x, y, heading)", "%.1f, %.1f, %.0f",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("Odometry (x, y, heading)", "%.1f, %.1f, %.0f",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }

    /**
     * Runs once when START is pressed. Builds the match routine and schedules it.
     */
    @Override
    public void start() {
        // Set the start pose once more: by now the Pinpoint's gyro recalibration from INIT has long finished, so
        // this one can't be lost, and the robot is still sitting at the start spot
        robot.drivetrain.setPose(startPose);
        RobotLog.ii("MatchAuto", "Start pose had to be set again %d times during INIT", startPoseResets);
        matchRoutine = sequential(
                markStep("shoot", startPose),
                // Sit still while shooting: the robot was placed at the start pose, so there's nothing to correct,
                // and holding the pose would drive the robot off if the odometry were wrong
                instant(() -> robot.drivetrain.stop()),
                shootAllBallsCommand(),
                markStep("leaveWall", leaveWallPose),
                robot.drivetrain.followPathCommand(straightPath(startPose, leaveWallPose)),
                markStep("park", parkingPose),
                robot.drivetrain.followPathCommand(straightPath(leaveWallPose, parkingPose)),
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
        telemetry.addData("Indexer", robot.shooter.isIndexerFeeding() ? "FEEDING" : "resting");
        telemetry.addData("Indexer off rest (deg)", "%.1f", robot.shooter.getIndexerErrorToRestDegrees());
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
                robot.shooter.isIndexerFeeding(), robot.shooter.getIndexerAngleDegrees(),
                robot.shooter.getIndexerErrorToRestDegrees(),
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
