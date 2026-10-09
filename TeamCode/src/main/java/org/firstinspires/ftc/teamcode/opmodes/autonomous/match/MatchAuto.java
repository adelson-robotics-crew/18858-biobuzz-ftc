package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.logging.CsvLog;
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.Alliance;

/**
 * What every match autonomous shares: setting and checking the start pose, running the routine as one Ivy command
 * from loop(), telemetry, and a CSV log of every loop on the Robot Controller (see CsvLog) so a run can be analyzed
 * afterward. Each routine (MatchAuto1, MatchAuto2) only gives its start pose and builds its command; the building
 * blocks it needs (straightPath(), markStep(), aimAndShootCommand()) are here.
 * Routines are written once with red coordinates, and blue runs them rotated 180 deg about the field center
 * (see forAlliance()). Not listed on the Driver Station itself: a thin Red/Blue wrapper per routine chooses the
 * alliance. Run Match TeleOp right after; it picks up the auto's final pose and alliance.
 */
public abstract class MatchAuto extends OpMode {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Every auto shot aims like the TeleOp's aim trigger (shot table RPM, heading aligned), see aimAndShootCommand().
    // How long it keeps shooting, counted from when the shot is first ready (aligned, in range, wheel at speed).
    private static final double AUTO_SHOOT_SECONDS = 4.0;
    // If the shot isn't ready by this long after aiming starts, the AUTO_SHOOT_SECONDS start anyway, so a wheel
    // that never settles can't keep the robot from parking. The indexer still only feeds while the shot is ready.
    private static final double AUTO_AIM_TIMEOUT_SECONDS = 3.0;

    protected Robot robot;

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

    // How many times init_loop() had to set the start pose again because the odometry didn't read it, shown on
    // telemetry (and logged) to tell whether the Pinpoint is losing the start pose
    private int startPoseResets = 0;

    // This alliance's version of the routine's start pose, set in init()
    private Pose startPose;

    /**
     * Which alliance this auto runs for. Picking the red or blue auto is how the robot knows its side: the auto
     * saves it (Robot.saveAlliance()) and the TeleOp that follows aims at that alliance's hive.
     *
     * @return the alliance
     */
    protected abstract Alliance alliance();

    /**
     * The routine's name, used for the log file name (with the alliance added).
     *
     * @return the name, e.g. "MatchAuto1"
     */
    protected abstract String routineName();

    /**
     * Where the robot is placed at the start, on the red side. The start heading must match how the robot is
     * really placed: telling the localizer the opposite heading turns every move around.
     *
     * @return the red start pose, in Pedro coordinates
     */
    protected abstract Pose redStartPose();

    /**
     * Builds the routine, run once from start(). Build every pose with forAlliance() so blue runs it rotated.
     *
     * @param allianceStartPose this alliance's start pose (redStartPose() run through forAlliance())
     * @return the whole routine as one command
     */
    protected abstract Command buildRoutine(Pose allianceStartPose);

    /**
     * Turns a pose written for the red side into this alliance's version of it.
     *
     * @param redSidePose a pose on the red side, in Pedro coordinates
     * @return the pose unchanged on red, rotated 180 deg about the field center on blue
     */
    protected Pose forAlliance(Pose redSidePose) {
        return alliance().fromRedSide(redSidePose);
    }

    /**
     * Builds a straight path whose heading turns evenly from pathStartPose's heading to pathEndPose's heading
     * as the robot moves along it (constant the whole way if the two headings match).
     *
     * @param pathStartPose where the path starts
     * @param pathEndPose   where the path ends
     * @return the straight path from pathStartPose to pathEndPose
     */
    protected Path straightPath(Pose pathStartPose, Pose pathEndPose) {
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
    protected Command markStep(String stepName, Pose targetPose) {
        return instant(() -> {
            currentStepName = stepName;
            currentStepTargetPose = targetPose;
        });
    }

    /**
     * Builds the shooting step every auto uses: the same aim-and-shoot as holding the TeleOp's aim trigger with the
     * sticks centered (see Robot.aimAndShootInPlace()), in order:
     *   1. Aim: turn in place to face the shooter at the alliance cell on the robot's half of the field, and spin the
     *      wheel to the shot table's RPM for the distance, until the shot is ready (valid position, heading aligned,
     *      wheel at speed), or AUTO_AIM_TIMEOUT_SECONDS run out.
     *   2. Shoot for AUTO_SHOOT_SECONDS, still aiming. The indexer only feeds while the shot stays ready.
     *   3. Stop the shooter, then wait until the indexer's PID has it back at rest (or its correction window runs
     *      out), so the next step, or the end of the auto, never leaves it off its saved 0 deg grid.
     *
     * @return a command that finishes once the shot is done and the indexer is back at rest
     */
    protected Command aimAndShootCommand() {
        return sequential(
                // Whichever finishes first; infinite() never does, so this aims every loop until shooting is done
                race(
                        infinite(() -> robot.aimAndShootInPlace()),
                        sequential(
                                // Whichever finishes first: the shot is ready, or the aim timeout runs out
                                race(
                                        waitUntil(() -> robot.isReadyToShoot()),
                                        waitMs(AUTO_AIM_TIMEOUT_SECONDS * 1000.0)
                                ),
                                waitMs(AUTO_SHOOT_SECONDS * 1000.0)
                        )
                ),
                instant(() -> {
                    robot.stopAimAndShoot(); // the indexer PID starts correcting it to rest on the next update
                    robot.drivetrain.stop(); // the aim left a turn power applied; stop it before the next step
                }),
                // Back at rest, or the correction window ran out (so a stuck indexer can't hold up the routine)
                waitUntil(() -> robot.indexer.isAtRest() || robot.indexer.getState() == Indexer.State.HOLDING)
        );
    }

    /**
     * Runs once when INIT is pressed. Clears any commands left over from a previous OpMode,
     * creates the robot, gives it the alliance's cells to aim at, and tells it where the robot starts.
     */
    @Override
    public void init() {
        // The Ivy scheduler is static and outlives OpModes, so clear anything a previous OpMode left behind
        Scheduler.reset();
        startPose = forAlliance(redStartPose());
        robot = new Robot(hardwareMap);
        Robot.saveAlliance(alliance()); // the TeleOp that follows aims at this alliance's hive
        robot.setShotTargets(alliance().leftCell, alliance().rightCell); // for aimAndShootCommand()
        robot.drivetrain.setPose(startPose);
        robot.update(); // apply the starting pose before the OpMode starts

        matchLog = new CsvLog(routineName() + alliance().label,
                "timeSec", "loopMs", "step",
                "x", "y", "headingDeg",
                "targetX", "targetY", "targetHeadingDeg",
                "velocityX", "velocityY", "turnRateDegPerSec",
                "followerMode", "followerBusy", "pathProgress",
                "pathPointX", "pathPointY", "pathPointHeadingDeg",
                "shooterRpm", "shooterTargetRpm", "shooterAtSpeed",
                "indexerState", "indexerAngleDeg", "indexerOffRestDeg",
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
        RobotLog.ii(routineName(), "Start pose had to be set again %d times during INIT", startPoseResets);
        matchRoutine = sequential(
                buildRoutine(startPose),
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
        telemetry.addData("Step", currentStepName);
        telemetry.addData("Routine running", Scheduler.isScheduled(matchRoutine));
        telemetry.addData("Robot (x, y, heading)", "%.1f, %.1f, %.0f",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())); // heading radians -> degrees
        telemetry.addData("Follower busy", robot.drivetrain.isBusy());
        telemetry.addData("Shooter RPM (target / actual)", "%.0f / %.0f",
                robot.shooter.getTargetRpm(), robot.shooter.getShooterRpm());
        telemetry.addData("Shooter at speed", robot.shooter.isAtSpeed());
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
                robot.indexer.getState().name(), robot.indexer.getAngleDegrees(),
                robot.indexer.getErrorToRestDegrees(),
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
