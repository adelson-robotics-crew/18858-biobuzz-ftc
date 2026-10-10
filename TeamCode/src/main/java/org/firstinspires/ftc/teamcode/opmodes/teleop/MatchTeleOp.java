package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.logging.CsvLog;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

/**
 * The match TeleOp: field-centric driving with Pedro Pathing, the intake, manual shooting, and aim-and-shoot at
 * the alliance's HIVE cell on the robot's half of the field (AIM_AND_SHOOT_BUTTON, see Robot.applyDriverControls()).
 * Not listed on the Driver Station itself: thin wrappers (AutoAllianceMatchTeleOp, listed as "Match TeleOp", and
 * DriveTest) only choose the alliance and the fallback starting pose.
 *
 * Starting pose: the pose the autonomous left the robot in, carried over automatically (see
 * Drivetrain.getSavedPose()). Only if no pose was saved since the app started (e.g. after a power cycle), the
 * wrapper's fixed starting pose. All coordinates are Pedro field coordinates.
 * Every loop is written to a CSV log on the Robot Controller (see CsvLog) so a run can be analyzed afterward.
 */
public abstract class MatchTeleOp extends OpMode {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Odometry sanity check. The robot's center can't get closer to a wall than about this (against a wall it reads
    // about 8.8 in from the bottom wall and 133.4 from the top, see the Match Auto start poses), so a pose closer
    // than that, minus ODOMETRY_OFF_SLACK_INCHES, means the odometry is wrong (e.g. pods skidded when the robot hit
    // a wall: one 10-09 run read x = -26). The aim trigger trusts the pose, so the driver is told to shoot manually.
    private static final double ROBOT_CENTER_MIN_DISTANCE_FROM_WALL_INCHES = 8.0;
    private static final double ODOMETRY_OFF_SLACK_INCHES = 3.0;
    // The pose has to be impossible for this many loops in a row (about 0.15 s) before the warning comes on, so a
    // one-off bad read doesn't trigger it: the Pinpoint once returned (0, 0, 0) for 2 loops mid-drive, then was fine
    private static final int ODOMETRY_OFF_MIN_LOOPS = 5;

    private Robot robot;

    // Set once the pose has been somewhere the robot can't physically be, and kept for the rest of the run: an
    // odometry error stays even after the numbers drift back inside the field
    private boolean odometryOff = false;
    // How many loops in a row the pose has been somewhere the robot can't be
    private int unreachablePoseLoops = 0;

    // The pose the robot was told it starts at in init(), or null if none was set; init_loop() checks the odometry
    // still reads it
    private Pose startPose;

    // Shown on telemetry for the whole run, so the driver can see where the pose came from and which hive is aimed at
    private String startPoseSourceText;
    private String allianceText;
    private String warningText = null; // null when there's nothing to warn about

    // Shooter diagnostics: the velocity PIDF gains the hub is really using (read back at INIT and START) next to the
    // ones Shooter asks for, so a mismatch shows on telemetry; and the shooter motor's current this loop, for the log
    private String shooterHubPidfText = "not read yet";
    private String shooterConfiguredPidfText = "not read yet";
    private double shooterAmpsThisLoop = 0.0;
    // The battery voltage this loop, for telemetry and the log
    private double batteryVoltsThisLoop = 0.0;

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
     * Whether the driver stands on their alliance's side of the field, so a blue TeleOp drives from the blue
     * driver's side (sticks turned 180 deg from the red side). A practice TeleOp whose coordinates already treat
     * 0 deg as "away from the driver" turns this off.
     *
     * @return true: the driver controls follow the alliance
     */
    protected boolean driverSideFollowsAlliance() {
        return true;
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
            allianceText = alliance.label + " (from the " + alliance.label + " Match Auto)";
        } else if (alliance != null) {
            allianceText = alliance.label;
            // A fixed alliance that disagrees with the auto that just ran is probably the wrong TeleOp
            if (useCarriedPose() && savedAlliance != null && savedAlliance != alliance) {
                warningText = "The last auto was " + savedAlliance.label + " but this is the " + alliance.label
                        + " TeleOp. Check the selection.";
            }
        } else {
            allianceText = "UNKNOWN";
            warningText = "No auto has run since the robot was restarted, so the alliance is unknown and the "
                    + "right trigger (aim and shoot) is off. A (manual shot) still works. "
                    + "Run a red or blue Match Auto first.";
        }

        robot = new Robot(hardwareMap);
        robot.setResetButtonResetsPosition(resetButtonResetsPosition());
        robot.setDriverOnBlueSide(driverSideFollowsAlliance() && alliance == Alliance.BLUE);
        if (alliance != null) {
            // Both cells go to the robot, which aims at whichever is on its half of the field. Move them into this
            // TeleOp's coordinates (no change for Pedro coordinates)
            double offsetInches = coordinateOffsetInches();
            robot.setShotTargets(alliance.leftCell.shifted(-offsetInches, -offsetInches),
                    alliance.rightCell.shifted(-offsetInches, -offsetInches));
        }
        if (startPose != null) {
            robot.drivetrain.setPose(startPose);
        }
        robot.update(); // apply the start pose before the OpMode starts
        readShooterPidf();

        teleOpLog = new CsvLog(getClass().getSimpleName(),
                "timeSec", "loopMs", "alliance",
                "aimHeld", "manualShotHeld", "intakeHeld", "mode",
                "x", "y", "headingDeg", "turnRateDegPerSec",
                "cell", "shotHeadingDeg", "headingErrorDeg", "aimTurnPower",
                "shotValidity", "shotDistanceIn", "angleOffCenterDeg",
                "shooterTargetRpm", "shooterRpm", "shooterAtSpeed", "shooterAmps", "batteryVolts",
                "readyToShoot", "robotState",
                "indexerState", "indexerAngleDeg", "indexerOffRestDeg", "indexerEncoderVolts", "odometryOff");

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
        readShooterPidf(); // again, in case anything changed the gains since INIT
        timeSinceStart.reset();
        previousLoopSeconds = 0.0;
    }

    /**
     * Runs repeatedly after START. Applies the driver controls, updates the robot, then reports to telemetry:
     * the shot position (IN RANGE / TOO FAR / ...) first, then the aimed shot's numbers while aiming, then the
     * robot's state and pose.
     */
    @Override
    public void loop() {
        robot.applyDriverControls(gamepad1);
        robot.update();

        Pose robotPose = robot.drivetrain.getPose(); // position from the Pinpoint; inches for x/y, radians for heading
        unreachablePoseLoops = isPoseReachable(robotPose) ? 0 : unreachablePoseLoops + 1;
        if (unreachablePoseLoops >= ODOMETRY_OFF_MIN_LOOPS) {
            odometryOff = true;
        }
        if (odometryOff) {
            telemetry.addData("!!!", "ODOMETRY OFF - USE MANUAL SHOOTING");
        }
        if (warningText != null) {
            telemetry.addData("WARNING", warningText);
        }

        // At the top, always: whether a shot from here would be in range, even before the aim trigger is pulled
        ShotSolution shotFromHere = robot.getShotSolutionFromHere(); // null if the alliance (so the cells) is unknown
        telemetry.addData("Shot Position",
                shotFromHere != null ? shotFromHere.validity.label : "UNKNOWN (no alliance)");

        ShotSolution shotSolution = robot.getShotSolution(); // null unless the aim trigger is held
        if (shotSolution != null) {
            telemetry.addData("Shot READY", robot.isReadyToShoot());
            telemetry.addData("Shot Cell", robot.getShotTarget().name);
            telemetry.addData("Shot Distance (in)", "%.1f", shotSolution.distanceInches);
            telemetry.addData("Shot Angle Off Center (deg)", "%.1f", shotSolution.angleOffCenterDegrees);
            telemetry.addData("Shot Heading Error (deg)", "%.1f", robot.getAimHeadingErrorDeg());
            telemetry.addData("Shot RPM (target / actual)", "%.0f / %.0f",
                    shotSolution.targetRpm, robot.shooter.getShooterRpm());
        } else if (RobotConstants.SHOOT_BUTTON.test(gamepad1)) {
            // Manual shot held: the flywheel numbers, for tuning its PID
            telemetry.addData("Manual RPM (target / actual)", "%.0f / %.0f",
                    robot.shooter.getTargetRpm(), robot.shooter.getShooterRpm());
            telemetry.addData("Shooter At Speed", robot.shooter.isAtSpeed());
        }

        telemetry.addData("Robot State", robot.getSuperState().label);
        telemetry.addData("Alliance", allianceText);
        telemetry.addData("Robot (x, y, heading)", "%.1f, %.1f, %.0f",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())); // heading radians -> degrees
        shooterAmpsThisLoop = robot.shooter.getMotorCurrentAmps();
        batteryVoltsThisLoop = robot.getBatteryVoltage();
        telemetry.addData("Shooter Current (A)", "%.2f", shooterAmpsThisLoop);
        telemetry.addData("Battery (V)", "%.2f", batteryVoltsThisLoop);
        telemetry.addData("Shooter PIDF on hub", shooterHubPidfText);
        telemetry.addData("Shooter PIDF asked for", shooterConfiguredPidfText);
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

        boolean manualShotHeld = RobotConstants.SHOOT_BUTTON.test(gamepad1);
        String mode;
        if (shotSolution != null) {
            mode = "aim";
        } else if (manualShotHeld) {
            mode = "manualShot";
        } else {
            mode = "drive";
        }

        teleOpLog.addRow(
                nowSeconds, loopMilliseconds, allianceText,
                RobotConstants.AIM_AND_SHOOT_BUTTON.test(gamepad1),
                manualShotHeld, RobotConstants.INTAKE_BUTTON.test(gamepad1), mode,
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
                shooterAmpsThisLoop, batteryVoltsThisLoop,
                robot.isReadyToShoot(), robot.getSuperState().label,
                robot.indexer.getState().name(), robot.indexer.getAngleDegrees(),
                robot.indexer.getErrorToRestDegrees(), robot.indexer.getEncoderVoltage(), odometryOff);
    }

    /**
     * Whether the robot could physically be at this pose: its center at least
     * ROBOT_CENTER_MIN_DISTANCE_FROM_WALL_INCHES (less the slack) from every wall. The field is square, twice the symmetry center across. A TeleOp in shifted
     * coordinates (coordinateOffsetInches()) is shifted back to Pedro first.
     *
     * @param robotPose the pose the odometry reports, in this TeleOp's coordinates
     * @return true if the pose is somewhere the robot can be; false means the odometry is off
     */
    private boolean isPoseReachable(Pose robotPose) {
        double fieldSizeInches = 2.0 * RobotConstants.FIELD_SYMMETRY_CENTER_X_INCHES;
        double nearestCenterInches = ROBOT_CENTER_MIN_DISTANCE_FROM_WALL_INCHES - ODOMETRY_OFF_SLACK_INCHES;
        double pedroX = robotPose.x() + coordinateOffsetInches();
        double pedroY = robotPose.y() + coordinateOffsetInches();
        return pedroX >= nearestCenterInches && pedroX <= fieldSizeInches - nearestCenterInches
                && pedroY >= nearestCenterInches && pedroY <= fieldSizeInches - nearestCenterInches;
    }

    /**
     * Reads back the shooter's velocity PIDF gains from the hub and the ones Shooter asks for, for telemetry and the
     * Robot Controller log. A hub read, so only done at INIT and START.
     */
    private void readShooterPidf() {
        shooterHubPidfText = formatPidf(robot.shooter.readHubVelocityPidf());
        PIDFCoefficients configured = robot.shooter.getConfiguredVelocityPidf();
        shooterConfiguredPidfText = configured == null ? "none (hub defaults)" : formatPidf(configured);
        RobotLog.ii("MatchTeleOp", "Shooter velocity PIDF on hub: %s, asked for: %s",
                shooterHubPidfText, shooterConfiguredPidfText);
    }

    /**
     * Formats PIDF gains for telemetry.
     *
     * @param gains the gains to show
     * @return e.g. "P 300.000 I 0.000 D 0.000 F 13.000 (LegacyPID)"
     */
    private static String formatPidf(PIDFCoefficients gains) {
        return String.format("P %.3f I %.3f D %.3f F %.3f (%s)",
                gains.p, gains.i, gains.d, gains.f, gains.algorithm);
    }

    /**
     * Adds the alliance, where the starting pose came from, and any warning to telemetry.
     */
    private void addStartInfoToTelemetry() {
        if (warningText != null) {
            telemetry.addData("WARNING", warningText);
        }
        telemetry.addData("Alliance", allianceText);
        telemetry.addData("Driver Side", robot.isDriverOnBlueSide() ? "Blue" : "Red");
        telemetry.addData("Start Pose", startPoseSourceText);
        telemetry.addData("Shooter PIDF on hub", shooterHubPidfText);
        telemetry.addData("Shooter PIDF asked for", shooterConfiguredPidfText);
    }
}
