package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.logging.CsvLog;
import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

/**
 * The match TeleOp: field-centric driving with Pedro Pathing, the intake, manual shooting, and aim-and-shoot at
 * the alliance's two HIVE cells (AIM_LEFT_CELL_BUTTON / AIM_RIGHT_CELL_BUTTON, see Robot.applyDriverControls()).
 * Not listed on the Driver Station itself: thin wrappers (AutoAllianceMatchTeleOp, listed as "Match TeleOp", and
 * DriveTest) only choose the alliance and the fallback starting pose.
 *
 * Starting pose: the pose the autonomous left the robot in, carried over automatically (see
 * Drivetrain.getSavedPose()). Only if no pose was saved since the app started (e.g. after a power cycle), the
 * wrapper's fixed starting pose. All coordinates are Pedro field coordinates.
 * Every loop is written to a CSV log on the Robot Controller (see CsvLog) so a run can be analyzed afterward.
 */
public abstract class MatchTeleOp extends OpMode {
    private Robot robot;

    // The pose the robot was told it starts at in init(), or null if none was set; init_loop() checks the odometry
    // still reads it
    private Pose startPose;

    // Shown on telemetry for the whole run, so the driver can see where the pose came from and which hive is aimed at
    private String startPoseSourceText;
    private String allianceText;
    private String warningText = null; // null when there's nothing to warn about

    // One row per loop, written to /sdcard/logs/ on the Robot Controller
    private CsvLog teleOpLog;
    // Time since START, for the log's time column; and the time of the previous loop, for measuring loop length
    private final ElapsedTime timeSinceStart = new ElapsedTime();
    private double previousLoopSeconds = 0.0;

    /**
     * The alliance this TeleOp aims for.
     *
     * @return the alliance, or null to work it out from which half of the field the carried-over pose is on
     */
    protected abstract Alliance fixedAlliance();

    /**
     * The pose to start from when no pose was carried over (no OpMode has run since the app started, e.g. after a
     * power cycle). The robot must really be there, or field-centric driving and aiming will be off.
     *
     * @return the fallback starting pose, or null to leave the localizer's pose as it is
     */
    protected abstract Pose fallbackStartPose();

    /**
     * Whether to start from the pose the previous OpMode left the robot in. A practice TeleOp that always starts
     * from a fixed spot turns this off.
     *
     * @return true to use the carried-over pose when there is one
     */
    protected boolean useCarriedPose() {
        return true;
    }

    /**
     * How far this TeleOp's coordinates are shifted from Pedro's: its (0, 0) is Pedro (offset, offset). The hive
     * cells are moved by the same amount so aiming still points at the right place.
     *
     * @return the offset in inches; 0 for Pedro coordinates
     */
    protected double coordinateOffsetInches() {
        return 0.0;
    }

    /**
     * Whether RESET_HEADING_BUTTON resets the whole pose to (0, 0, 0 deg) instead of only the heading.
     *
     * @return false: in a match only the heading is reset
     */
    protected boolean resetButtonResetsPosition() {
        return false;
    }

    /**
     * Runs once when INIT is pressed. Works out the starting pose and the alliance, creates the robot (which
     * initializes all hardware), gives it the alliance's HIVE cells, and sets the pose.
     */
    @Override
    public void init() {
        // Read before anything calls robot.update(), which saves this OpMode's own pose over it
        Pose carriedPose = useCarriedPose() ? Drivetrain.getSavedPose() : null;

        if (carriedPose != null) {
            startPose = carriedPose;
            startPoseSourceText = "From previous OpMode (auto)";
        } else {
            startPose = fallbackStartPose();
            startPoseSourceText = startPose != null ? "Fixed start pose" : "UNKNOWN (pose not set)";
        }

        Alliance savedAlliance = Robot.getSavedAlliance(); // set by the last autonomous that ran
        Alliance alliance = fixedAlliance();
        if (alliance == null && savedAlliance != null) {
            alliance = savedAlliance;
            allianceText = alliance.label + " (from Match Auto " + alliance.label + ")";
        } else if (alliance != null) {
            allianceText = alliance.label;
            // A fixed alliance that disagrees with the auto that just ran is probably the wrong TeleOp
            if (useCarriedPose() && savedAlliance != null && savedAlliance != alliance) {
                warningText = "The last auto was " + savedAlliance.label + " but this is the " + alliance.label
                        + " TeleOp. Check the selection.";
            }
        } else {
            allianceText = "UNKNOWN";
            warningText = "No auto has run since the robot was restarted, so the alliance is unknown and "
                    + "X/B aiming is off. Run Match Auto Red or Match Auto Blue first.";
        }

        robot = new Robot(hardwareMap);
        robot.setResetButtonResetsPosition(resetButtonResetsPosition());
        if (alliance != null) {
            // Move the cells into this TeleOp's coordinates (no change for Pedro coordinates)
            double offsetInches = coordinateOffsetInches();
            robot.setShotTargets(alliance.leftCell.shifted(-offsetInches, -offsetInches),
                    alliance.rightCell.shifted(-offsetInches, -offsetInches));
        }
        if (startPose != null) {
            robot.drivetrain.setPose(startPose);
        }
        robot.update(); // apply the start pose before the OpMode starts

        teleOpLog = new CsvLog(getClass().getSimpleName(),
                "timeSec", "loopMs", "alliance",
                "xHeld", "bHeld", "rightTriggerHeld", "leftTriggerHeld", "mode",
                "x", "y", "headingDeg", "turnRateDegPerSec",
                "cell", "shotHeadingDeg", "headingErrorDeg", "aimTurnPower",
                "shotValidity", "shotDistanceIn", "angleOffCenterDeg",
                "shooterTargetRpm", "shooterRpm", "shooterAtSpeed", "readyToShoot", "robotState",
                "indexerFeeding", "indexerAngleDeg", "indexerOffRestDeg", "indexerEncoderVolts");

        addStartInfoToTelemetry();
        telemetry.update();
    }

    /**
     * Runs repeatedly between INIT and START. Keeps the odometry updating and shows whether it still reads the
     * starting pose, so a broken odometry pod or Pinpoint is caught before driving instead of during it.
     */
    @Override
    public void init_loop() {
        robot.update();
        // Creating the robot recalibrates the Pinpoint's gyro (about a quarter second), and a start pose set while
        // that's still running can be lost; if the odometry doesn't read it, set it again
        if (startPose != null && !robot.drivetrain.isAtPose(startPose)) {
            robot.drivetrain.setPose(startPose);
        }
        addStartInfoToTelemetry();
        Pose robotPose = robot.drivetrain.getPose();
        if (startPose != null && !robot.drivetrain.isAtPose(startPose)) {
            telemetry.addData("ODOMETRY ERROR", "Doesn't read the start pose even after setting it again. Check the "
                    + "Pinpoint and pod cables, then power-cycle the robot.");
        } else if (startPose != null) {
            telemetry.addData("Odometry", "OK: reads the start pose");
        }
        telemetry.addData("Odometry (x, y, heading)", "%.1f, %.1f, %.0f",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }

    /**
     * Runs once when START is pressed. Starts the log's clock.
     */
    @Override
    public void start() {
        // Set the start pose once more, now that the Pinpoint's gyro recalibration from INIT has finished
        if (startPose != null) {
            robot.drivetrain.setPose(startPose);
        }
        timeSinceStart.reset();
        previousLoopSeconds = 0.0;
    }

    /**
     * Runs repeatedly after START. Applies the driver controls, updates the robot, then reports the robot
     * pose, shooter, and (while aiming) shot numbers to telemetry.
     */
    @Override
    public void loop() {
        robot.applyDriverControls(gamepad1);
        robot.update();

        addStartInfoToTelemetry();
        Pose robotPose = robot.drivetrain.getPose(); // position from the Pinpoint; inches for x/y, radians for heading
        telemetry.addData("Robot X (in)", robotPose.x());
        telemetry.addData("Robot Y (in)", robotPose.y());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotPose.heading())); // radians -> degrees for reading
        telemetry.addData("Heading Locked", robot.drivetrain.isHeadingLocked());
        telemetry.addData("Robot State", robot.getSuperState().label);
        telemetry.addData("Manual Shot RPM", "%.0f", robot.getManualShotRpm());
        telemetry.addData("Indexer", robot.shooter.isIndexerFeeding() ? "FEEDING" : "resting");
        telemetry.addData("Indexer Angle (deg)", "%.1f", robot.shooter.getIndexerAngleDegrees());
        telemetry.addData("Indexer Off Rest (deg)", "%.1f", robot.shooter.getIndexerErrorToRestDegrees());
        telemetry.addData("Indexer Encoder (V)", "%.3f", robot.shooter.getIndexerEncoderVoltage());

        ShotSolution shotSolution = robot.getShotSolution(); // null unless an aim button is held
        if (shotSolution != null) {
            telemetry.addData("Shot Cell", robot.getShotTarget().name);
            telemetry.addData("Shot READY", robot.isReadyToShoot());
            telemetry.addData("Shot Position", shotSolution.validity.label);
            telemetry.addData("Shot Distance (in)", "%.1f", shotSolution.distanceInches);
            telemetry.addData("Shot Angle Off Center (deg)", "%.1f", shotSolution.angleOffCenterDegrees);
            telemetry.addData("Shot Target Heading (deg)", "%.1f", Math.toDegrees(shotSolution.targetHeadingRadians));
            telemetry.addData("Shot Heading Error (deg)", "%.1f", robot.getAimHeadingErrorDeg());
            telemetry.addData("Shot Target RPM", "%.0f", shotSolution.targetRpm);
            telemetry.addData("Shot Actual RPM", "%.0f", robot.shooter.getShooterRpm());
        }
        telemetry.addData("Log file", teleOpLog.getFilePath());
        if (teleOpLog.getErrorMessage() != null) {
            telemetry.addData("Log error", teleOpLog.getErrorMessage());
        }

        telemetry.update();

        logThisLoop(robotPose, shotSolution);
    }

    /**
     * Runs once when STOP is pressed. Stops the shooter and closes the log.
     */
    @Override
    public void stop() {
        robot.stopShoot();
        robot.update(); // apply the stop right away
        teleOpLog.close(); // writes out the rows still buffered
    }

    /**
     * Writes one row to the TeleOp log describing this loop: the driver's shooting buttons, what the robot did with
     * them, and (while aiming) the shot and how the turn toward it is going. Call after robot.update().
     *
     * @param robotPose    the robot's pose this loop
     * @param shotSolution the shot this loop, or null while not aiming
     */
    private void logThisLoop(Pose robotPose, ShotSolution shotSolution) {
        double nowSeconds = timeSinceStart.seconds();
        double loopMilliseconds = (nowSeconds - previousLoopSeconds) * 1000.0;
        previousLoopSeconds = nowSeconds;

        boolean rightTriggerHeld = RobotConstants.SHOOT_BUTTON.test(gamepad1);
        String mode;
        if (shotSolution != null) {
            mode = "aim";
        } else if (rightTriggerHeld) {
            mode = "manualShot";
        } else {
            mode = "drive";
        }

        teleOpLog.addRow(
                nowSeconds, loopMilliseconds, allianceText,
                RobotConstants.AIM_LEFT_CELL_BUTTON.test(gamepad1), RobotConstants.AIM_RIGHT_CELL_BUTTON.test(gamepad1),
                rightTriggerHeld, RobotConstants.INTAKE_BUTTON.test(gamepad1), mode,
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()),
                Math.toDegrees(robot.drivetrain.getVelocity().omega),
                shotSolution == null ? "" : robot.getShotTarget().name,
                shotSolution == null ? "" : Math.toDegrees(shotSolution.targetHeadingRadians),
                shotSolution == null ? "" : robot.getAimHeadingErrorDeg(),
                shotSolution == null ? "" : robot.drivetrain.getLastAimTurnPower(),
                shotSolution == null ? "" : shotSolution.validity.label,
                shotSolution == null ? "" : shotSolution.distanceInches,
                shotSolution == null ? "" : shotSolution.angleOffCenterDegrees,
                robot.shooter.getTargetRpm(), robot.shooter.getShooterRpm(), robot.shooter.isAtSpeed(),
                robot.isReadyToShoot(), robot.getSuperState().label,
                robot.shooter.isIndexerFeeding(), robot.shooter.getIndexerAngleDegrees(),
                robot.shooter.getIndexerErrorToRestDegrees(), robot.shooter.getIndexerEncoderVoltage());
    }

    /**
     * Adds the alliance, where the starting pose came from, and any warning to telemetry.
     */
    private void addStartInfoToTelemetry() {
        if (warningText != null) {
            telemetry.addData("WARNING", warningText);
        }
        telemetry.addData("Alliance", allianceText);
        telemetry.addData("Start Pose", startPoseSourceText);
    }
}
