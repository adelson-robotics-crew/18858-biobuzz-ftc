package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.pedro.Constants;

/**
 * The drivetrain and its localization (the Pinpoint). This class is the only one that touches
 * the Pedro follower; everything else drives and reads the robot's pose through these methods,
 * or gets Ivy commands from followPathCommand() / holdPoseCommand() that wrap the follower internally.
 * Those commands also stop the drivetrain and finish if the robot stalls (e.g. drives into a wall), see StallDetector.
 */
public class Drivetrain {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // A drive command (follow a path, hold a pose) counts as finished once the robot is within
    // these distances of its target pose, in both position and heading. The same tolerances are used at INIT to
    // check that the odometry really took the starting pose (see Drivetrain.isAtPose()).
    private static final double DRIVE_POSITION_TOLERANCE_INCHES = 1.0;
    private static final double DRIVE_HEADING_TOLERANCE_DEGREES = 3.0;

    // Stall detection (every drive command, e.g. autonomous legs). If the robot hasn't moved
    // more than DRIVE_STALL_MOVEMENT_INCHES or turned more than DRIVE_STALL_TURN_DEGREES for DRIVE_STALL_SECONDS
    // while a command is driving it, it's assumed to be pushing into a wall: the drivetrain stops and the command
    // counts as finished, as if the robot had reached its target.
    private static final double DRIVE_STALL_SECONDS = 1.0;
    private static final double DRIVE_STALL_MOVEMENT_INCHES = 0.5;
    private static final double DRIVE_STALL_TURN_DEGREES = 2.0;

    // Heading lock (TeleOp). While the turn stick is at or below HEADING_LOCK_TURN_THRESHOLD, the robot holds
    // its heading instead of drifting; pushing the stick past it turns the lock off so the driver turns freely.
    // Set HEADING_LOCK_ENABLED to false to drive exactly as before the lock existed.
    private static final boolean HEADING_LOCK_ENABLED = false;
    // Turn stick magnitude (0 to 1) at or below which the stick counts as released and the lock engages
    private static final double HEADING_LOCK_TURN_THRESHOLD = 0.05;
    // After the turn stick is released, the lock waits until the robot is rotating slower than this before it
    // grabs the heading to hold. Grabbing it right away would pull the robot back against its own turning momentum.
    private static final double HEADING_LOCK_SETTLE_DEGREES_PER_SECOND = 20.0;
    // PD gains of the heading lock: turn power per radian of heading error, and turn power per rad/s of rotation
    // (the D term damps the correction so it doesn't overshoot). Raise P if the robot gets knocked off its
    // heading too easily; raise D if it wobbles back and forth around the locked heading.
    private static final double HEADING_LOCK_P = 0.2;
    private static final double HEADING_LOCK_D = 0.05;

    // Aiming turn controller (right trigger held): turn power = HEADING_KP * error - HEADING_KD * turn rate, plus HEADING_KS
    // in the direction of the error while the error is bigger than HEADING_KS_DEADBAND_DEG, clamped to [-1, 1].
    // P alone (the old controller) asked for too little power near the target to overcome friction, so the robot
    // stalled a few degrees short of HEADING_TOLERANCE_DEG and never fed. Tune on the robot with the TeleOp log:
    //   stops short of the target -> raise HEADING_KS;  wobbles back and forth -> raise HEADING_KD or lower HEADING_KP;
    //   turns too slowly from far away -> raise HEADING_KP.
    // Turn power per radian of heading error
    private static final double HEADING_KP = 1.2;
    // Turn power per radian/second of turning, subtracted to brake the turn as it nears the target
    private static final double HEADING_KD = 0.06;
    // Minimum turn power that overcomes friction, added in the direction of the error
    private static final double HEADING_KS = 0.07;
    // Below this much error, HEADING_KS is left off so the robot doesn't jitter around the target. Must be smaller
    // than HEADING_TOLERANCE_DEG so the robot still gets inside the tolerance.
    private static final double HEADING_KS_DEADBAND_DEG = 1.0;

    // The robot's pose as of the last update() in any OpMode. Static so it survives from one OpMode to the next
    // (e.g. autonomous into TeleOp) while the app keeps running; a power cycle or app restart clears it.
    // Read it with getSavedPose()
    private static Pose lastSavedPose = null;

    private final Follower follower;

    // Turns heading error into turn power for the TeleOp heading lock (no I term: the D term uses the measured rotation speed)
    private final PIDController headingLockController =
            new PIDController(HEADING_LOCK_P, 0.0, HEADING_LOCK_D);
    // The heading (radians) the lock is holding, or null while unlocked (driver turning, or robot still settling)
    private Double lockedHeadingRadians = null;

    // The turn power the aiming controller used last, kept for logging
    private double lastAimTurnPower = 0.0;

    /**
     * Creates the Pedro follower, which sets up the motors and the Pinpoint localizer.
     *
     * @param hardwareMap the hardware map from the running OpMode
     */
    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    /**
     * Runs the follower. Must be called every loop, including while driving manually.
     */
    public void update() {
        follower.update();
        // Save the pose every loop, so the next OpMode can pick up where this one left off
        lastSavedPose = follower.pose();
    }

    /**
     * The robot's pose as the previous OpMode last saw it (normally where the autonomous ended). Read this in
     * init() before calling update(), which would overwrite it with the new OpMode's pose.
     *
     * @return the saved pose, or null if no OpMode has run since the app started (e.g. after a power cycle)
     */
    public static Pose getSavedPose() {
        return lastSavedPose;
    }

    /**
     * Drives with field-centric controls: "forward" is away from the driver no matter which way the robot faces.
     * With the heading lock on (HEADING_LOCK_ENABLED), a turn input at or below HEADING_LOCK_TURN_THRESHOLD
     * makes the robot hold its heading; a larger turn input releases the lock and turns the robot as asked.
     *
     * @param forward the requested forward power, from -1 to 1
     * @param strafe  the requested sideways power, from -1 to 1
     * @param turn    the requested turning power, from -1 to 1
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        // A path or hold (e.g. an autonomous) ran since the last manual drive and may have turned the
        // robot, so the locked heading is stale; holding it would spin the robot back to where it was before
        if (!follower.manual()) {
            releaseHeadingLock();
        }

        boolean driverIsTurning = Math.abs(turn) > HEADING_LOCK_TURN_THRESHOLD;
        if (!HEADING_LOCK_ENABLED || driverIsTurning) {
            releaseHeadingLock();
            follower.manual(ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    follower.pose().heading() // the Pinpoint heading is what rotates the input into the field frame
            ));
            return;
        }

        // Turn stick released: translate with no turn input, then let the lock supply the turn power
        DrivePowers translationOnlyPowers = ManualDrive.fieldCentric(forward, strafe, 0.0, follower.pose().heading());

        if (lockedHeadingRadians == null) {
            double rotationSpeedDegreesPerSecond = Math.abs(Math.toDegrees(follower.velocity().omega));
            if (rotationSpeedDegreesPerSecond >= HEADING_LOCK_SETTLE_DEGREES_PER_SECOND) {
                // Still coasting from the last turn; let it slow down before picking the heading to hold
                follower.manual(translationOnlyPowers);
                return;
            }
            lockedHeadingRadians = follower.pose().heading();
            headingLockController.reset(); // clear the integral and timing left over from the previous lock
        }

        // Replaces the turn power with PD feedback on the heading error, keeping forward and strafe as they are
        follower.manual(ManualDrive.headingLock(follower, headingLockController, translationOnlyPowers, lockedHeadingRadians));
    }

    /**
     * Drives with field-centric translation from the driver while the robot turns itself to a target heading
     * (e.g. to aim the shooter). Unlike holdPose(), this doesn't lock position: the driver keeps full control of
     * forward and strafe. Turn power comes from aimTurnPower().
     *
     * @param forward              the requested forward power, from -1 to 1
     * @param strafe               the requested sideways power, from -1 to 1
     * @param targetHeadingRadians the heading to turn to and hold
     */
    public void driveFieldCentricWithHeading(double forward, double strafe, double targetHeadingRadians) {
        // The heading lock isn't used here, and its old heading would be stale once this turns the robot
        releaseHeadingLock();
        double currentHeadingRadians = follower.pose().heading();
        lastAimTurnPower = aimTurnPower(currentHeadingRadians, targetHeadingRadians);
        follower.manual(ManualDrive.fieldCentric(forward, strafe, lastAimTurnPower, currentHeadingRadians));
    }

    /**
     * The turn power that turns the robot toward a target heading while aiming: proportional to the heading error,
     * braked by how fast the robot is already turning, plus a minimum push to overcome friction (see the HEADING_*
     * constants at the top of this class). Positive turn power turns counterclockwise, which is the way the heading
     * grows.
     *
     * @param currentHeadingRadians the robot's heading now
     * @param targetHeadingRadians  the heading to turn to
     * @return the turn power, from -1 to 1
     */
    private double aimTurnPower(double currentHeadingRadians, double targetHeadingRadians) {
        // Angle.error is (target - current) wrapped into [-PI, PI), so it's positive when the target is
        // counterclockwise of the robot, the same sign Pedro's own heading lock uses
        double headingErrorRadians = Angle.error(currentHeadingRadians, targetHeadingRadians);
        double turnRateRadiansPerSecond = follower.velocity().omega; // positive while turning counterclockwise
        double turnPower = HEADING_KP * headingErrorRadians
                - HEADING_KD * turnRateRadiansPerSecond;
        if (Math.abs(Math.toDegrees(headingErrorRadians)) > HEADING_KS_DEADBAND_DEG) {
            turnPower += Math.signum(headingErrorRadians) * HEADING_KS;
        }
        return Math.max(-1.0, Math.min(1.0, turnPower));
    }

    /**
     * The turn power the last driveFieldCentricWithHeading() call used, for logging.
     *
     * @return the last aiming turn power, from -1 to 1
     */
    public double getLastAimTurnPower() {
        return lastAimTurnPower;
    }

    /**
     * How far the robot's heading is from a target heading, wrapped so it never reads more than 180 deg.
     *
     * @param targetHeadingRadians the heading to compare against
     * @return target minus current heading, in degrees, in [-180, 180)
     */
    public double headingErrorDeg(double targetHeadingRadians) {
        return Math.toDegrees(Angle.error(follower.pose().heading(), targetHeadingRadians));
    }

    /**
     * Tells whether the heading lock is currently holding a heading.
     *
     * @return true while the robot is holding a locked heading; false while the driver turns or the robot settles
     */
    public boolean isHeadingLocked() {
        return lockedHeadingRadians != null;
    }

    /**
     * Forgets the locked heading. The next driveFieldCentric() call with the turn stick released grabs a fresh one.
     */
    private void releaseHeadingLock() {
        lockedHeadingRadians = null;
    }

    /**
     * Starts following a path. The follower keeps following it as update() is called.
     *
     * @param path the path to follow
     */
    public void followPath(Path path) {
        follower.follow(path);
    }

    /**
     * Holds the robot at a pose, correcting for both position and heading.
     *
     * @param targetPose the pose to hold
     */
    public void holdPose(Pose targetPose) {
        follower.hold(targetPose);
    }

    /**
     * Builds a command that follows a path and finishes only once the robot has actually arrived.
     * Ivy's own follow command finishes at follower.atParametricEnd(), which can be true before the
     * robot has settled at the end pose, so this waits for the same arrival check as holdPoseCommand().
     * The follower is not updated by the command; Robot.update() must still run every loop.
     * If the robot stalls on the way (see StallDetector), the drivetrain stops and the command finishes early.
     *
     * @param path the path to follow
     * @return a command that finishes when the follower is idle and the robot is at the path's end pose, or it stalls
     */
    public Command followPathCommand(Path path) {
        return sequential(
                PedroCommands.follow(follower, path).requiring(this),
                waitUntilArrivedOrStalled(path.endPose())
        );
    }

    /**
     * Builds a command that drives the robot to a pose and finishes once it gets there. The follower
     * keeps holding the pose after the command finishes, until another drive command replaces it.
     * Ivy's own hold command is an instant (it finishes the same loop it starts), so without the
     * wait below a sequence would move on before the robot has turned or moved at all.
     * If the robot stalls on the way (see StallDetector), the drivetrain stops (it no longer holds the pose)
     * and the command finishes early.
     *
     * @param targetPose the pose to drive to and hold
     * @return a command that finishes when the follower is idle and the robot is at the target pose, or it stalls
     */
    public Command holdPoseCommand(Pose targetPose) {
        return sequential(
                PedroCommands.hold(follower, targetPose).requiring(this),
                waitUntilArrivedOrStalled(targetPose)
        );
    }

    /**
     * Builds the waiting half of a drive command: finishes once the robot arrives at the target pose, or once
     * it has stalled, in which case the drivetrain is stopped first so it doesn't keep pushing into whatever
     * is blocking it. A stall counts as arriving, so a sequence moves on to its next step either way.
     *
     * @param targetPose the pose the drive command is heading for
     * @return a command that finishes on arrival or on a stall
     */
    private Command waitUntilArrivedOrStalled(Pose targetPose) {
        StallDetector stallDetector = new StallDetector(); // one per command, so each drive gets its own clock
        return sequential(
                // Start the stall clock when the wait starts (not when the command was built), from where the robot is then
                instant(() -> stallDetector.reset(follower.pose())),
                waitUntil(() -> {
                    if (!isBusy() && isAtPose(targetPose)) {
                        return true;
                    }
                    if (stallDetector.isStalled(follower.pose())) {
                        follower.stop(); // idle mode: follower.update() turns the drive motors off
                        return true;
                    }
                    return false;
                })
        );
    }

    /**
     * Stops the drivetrain: the follower goes idle and update() turns the drive motors off, so the robot sits still
     * (e.g. while the autonomous shoots) without trying to correct its position.
     */
    public void stop() {
        follower.stop();
    }

    /**
     * Tells whether the robot is close enough to a pose, in both position and heading, using the
     * drive tolerances at the top of this class. Checking position alone is what let an earlier autonomous
     * start its next leg before a turn was done, so drive commands finish on this instead.
     *
     * @param targetPose the pose the robot should be at
     * @return true if the robot is within the position and heading tolerances of the target
     */
    public boolean isAtPose(Pose targetPose) {
        Pose currentPose = follower.pose();
        double positionErrorInches = currentPose.distance(targetPose);
        // Angle.error wraps the difference into [-180, 180) deg (as radians), so 359 deg vs 1 deg reads as 2 deg
        double headingErrorDegrees = Math.toDegrees(Angle.error(currentPose.heading(), targetPose.heading()));
        return positionErrorInches < DRIVE_POSITION_TOLERANCE_INCHES
                && Math.abs(headingErrorDegrees) < DRIVE_HEADING_TOLERANCE_DEGREES;
    }

    /**
     * Tells whether the follower is still following a path.
     *
     * @return true while a path is being followed
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * The robot's measured velocity, for logging.
     *
     * @return field-frame velocity: vx and vy in inches per second, omega in radians per second
     */
    public Velocity getVelocity() {
        return follower.velocity();
    }

    /**
     * What the follower is doing (following a path, holding a pose, manual, idle), for logging.
     *
     * @return the follower mode's name
     */
    public String getFollowerModeName() {
        return follower.mode().name();
    }

    /**
     * How far along the current path the robot is, as Pedro measures it: the progress value t of the closest
     * point on the path, 0 at the start and 1 at the end. Heading interpolation (e.g. linear) uses this t.
     *
     * @return the current path's parametric completion, from 0 to 1, or NaN while not following a path
     */
    public double getPathProgress() {
        // Only meaningful (and only safe to ask for) while a path is being followed, not while holding or idle
        return follower.following() ? follower.parametricCompletion() : Double.NaN;
    }

    /**
     * The point on the current path closest to the robot, with the heading the path wants there.
     * This is what the follower is steering toward each loop.
     *
     * @return the closest path pose, with x and y in inches and heading in radians, or null while not following a path
     */
    public Pose getClosestPathPose() {
        // Only meaningful (and only safe to ask for) while a path is being followed, not while holding or idle
        return follower.following() ? follower.closestPose() : null;
    }

    /**
     * Pedro's full debug snapshot for this loop (follow state, localizer, drivetrain motor powers, and the
     * path-following algorithm's internals), as text. Meant for logs, not for logic: the keys inside are Pedro's
     * and can change between Pedro versions. Call after update() so it describes this loop.
     *
     * @return the snapshot as text
     */
    public String getFollowerDebugText() {
        return follower.debug().toString();
    }

    /**
     * Gets the robot's pose from the localizer.
     *
     * @return the pose, with x and y in inches and heading in radians
     */

    public Pose getPose() {
        return follower.pose();
    }

    /**
     * Detects a stalled robot: one that's being driven but hasn't moved or turned more than the
     * DRIVE_STALL_* thresholds for DRIVE_STALL_SECONDS. Movement is measured
     * from a reference pose, which moves up to the robot's pose (restarting the clock) whenever the robot gets
     * past either threshold, so slow but steady progress never counts as a stall.
     */
    private static class StallDetector {
        private Pose referencePose = null; // where the robot was when the clock last restarted
        private final ElapsedTime timeSinceLastMovement = new ElapsedTime();

        /**
         * Restarts the clock from the given pose. Call when the drive starts.
         *
         * @param currentPose the robot's pose now
         */
        void reset(Pose currentPose) {
            referencePose = currentPose;
            timeSinceLastMovement.reset();
        }

        /**
         * Checks for a stall. Call once per loop while driving.
         *
         * @param currentPose the robot's pose now
         * @return true once the robot has gone DRIVE_STALL_SECONDS without moving or turning past the thresholds
         */
        boolean isStalled(Pose currentPose) {
            if (referencePose == null) {
                reset(currentPose);
                return false;
            }
            double movedInches = currentPose.distance(referencePose);
            // Angle.error wraps the difference so turning across 0/360 deg reads as a small turn
            double turnedDegrees = Math.abs(Math.toDegrees(Angle.error(currentPose.heading(), referencePose.heading())));
            if (movedInches > DRIVE_STALL_MOVEMENT_INCHES
                    || turnedDegrees > DRIVE_STALL_TURN_DEGREES) {
                reset(currentPose); // still making progress
                return false;
            }
            return timeSinceLastMovement.seconds() > DRIVE_STALL_SECONDS;
        }
    }

    /**
     * Makes the robot's current facing the given heading, leaving x and y alone. Used to re-zero the heading with
     * the robot facing away from the driver: 0 deg for the red driver, 180 deg for the blue driver.
     * Needed because the Pinpoint keeps its heading between OpModes; it is only cleared on power-up.
     *
     * @param headingRadians the heading the robot's current facing becomes, in radians
     */
    public void resetHeading(double headingRadians) {
        follower.setHeading(headingRadians);
        releaseHeadingLock(); // the old locked heading was measured in the old frame
    }

    /**
     * Overrides where the drivetrain thinks the robot is, e.g. to set the starting pose of an autonomous.
     *
     * @param pose the pose to set, with x and y in inches and heading in radians
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
        releaseHeadingLock(); // the old locked heading was measured in the old frame
    }
}
