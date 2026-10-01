package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

/**
 * Owns every subsystem and is the single place hardware gets initialized. It also holds the logic
 * that ties the subsystems together: reading the gamepad through the mapping in RobotConstants,
 * and deciding which mechanisms may run together (the intake and shooter never do).
 * OpModes create one Robot in init() and talk to the subsystems through it;
 * they never touch hardware objects directly.
 */
public class Robot {

    /**
     * The robot's overall state. Each constant explicitly says what every subsystem's state
     * must be for the robot to be considered "in" that super state — this is the source of
     * truth a future command-based framework (e.g. Ivy) can validate or enforce transitions against.
     * As more subsystems and states are added, add a field to the constructor below for each
     * subsystem and give every constant a value for it.
     */
    public enum RobotSuperState {
        IDLE("Idle", Intake.State.IDLE, Shooter.State.IDLE),
        INTAKING("Intaking", Intake.State.INTAKING, Shooter.State.IDLE),
        SHOOTING("Shooting", Intake.State.IDLE, Shooter.State.SHOOTING);

        public final String label;
        public final Intake.State intakeState;
        public final Shooter.State shooterState;

        /**
         * @param label        the human-readable name shown in telemetry
         * @param intakeState  the state the intake must be in for the robot to be in this super state
         * @param shooterState the state the shooter must be in for the robot to be in this super state
         */
        RobotSuperState(String label, Intake.State intakeState, Shooter.State shooterState) {
            this.label = label;
            this.intakeState = intakeState;
            this.shooterState = shooterState;
        }

        /**
         * Looks up the super state with a given label, case-insensitively.
         * (Java enums already convert to/from their constant name for free, via name() and
         * valueOf(); this is only for matching the friendlier label instead, e.g. "Intaking".)
         *
         * @param label the label to look up
         * @return the matching super state, or null if no super state has that label
         */
        public static RobotSuperState fromLabel(String label) {
            for (RobotSuperState state : values()) {
                if (state.label.equalsIgnoreCase(label)) {
                    return state;
                }
            }
            return null;
        }
    }

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;

    // Whether each shooter-RPM button was held last loop, so a hold counts as one press instead of one per loop
    private boolean shooterRpmUpWasPressed = false;
    private boolean shooterRpmDownWasPressed = false;
    private boolean autoShootWasPressed = false;
    private boolean zeroPoseWasPressed = false;

    // Where the auto-shoot routine drives to and how it gets there, set by the OpMode through configureAutoShoot().
    // Null until then, and while null the auto-shoot button does nothing.
    private Pose autoShootShootingPose = null;
    private double autoShootWallClearanceYInches;
    private double autoShootDurationSeconds;

    // The drive-to-shooting-pose-and-shoot routine while it runs, or null before the first press.
    // Built fresh on each press, since its first leg starts from wherever the robot is at that moment.
    private Command autoShootRoutine = null;

    /**
     * Initializes all hardware by constructing each subsystem. Add new subsystems here.
     *
     * @param hardwareMap the hardware map from the running OpMode
     */
    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    /**
     * Updates every subsystem. Call this exactly once per OpMode loop.
     */
    public void update() {
        drivetrain.update();
        intake.update();
        shooter.update();
    }

    /**
     * Reads the driver gamepad through the mapping in RobotConstants and sends the resulting
     * commands to the subsystems. Call once per TeleOp loop, before Scheduler.execute() and update().
     * While the auto-shoot routine runs, it owns the drivetrain, intake, and shooter, so the sticks and
     * mechanism buttons are ignored; only the RPM bumpers and the auto-shoot (cancel) button still work.
     *
     * @param driverGamepad the gamepad that drives the robot (gamepad1 in the OpMode)
     */
    public void applyDriverControls(Gamepad driverGamepad) {
        // Start the auto-shoot routine on a press, or cancel it if it's already running
        boolean autoShootPressed = RobotConstants.AUTO_SHOOT_BUTTON.test(driverGamepad);
        if (autoShootPressed && !autoShootWasPressed) {
            if (isAutoShooting()) {
                cancelAutoShoot();
            } else {
                startAutoShoot();
            }
        }
        autoShootWasPressed = autoShootPressed;

        // Reset the localizer to the zero pose once per press. The auto-shoot routine's legs were planned
        // from the old pose, so it's cancelled rather than left driving toward targets that no longer line up.
        boolean zeroPosePressed = RobotConstants.ZERO_POSE_BUTTON.test(driverGamepad);
        if (zeroPosePressed && !zeroPoseWasPressed) {
            cancelAutoShoot();
            drivetrain.setPose(zeroPose());
        }
        zeroPoseWasPressed = zeroPosePressed;

        // Bump the shooter target RPM once per press, only on the loop the button goes down
        boolean shooterRpmUpPressed = RobotConstants.SHOOTER_RPM_UP_BUTTON.test(driverGamepad);
        if (shooterRpmUpPressed && !shooterRpmUpWasPressed) {
            shooter.adjustTargetRpm(RobotConstants.SHOOTER_RPM_ADJUST_STEP);
        }
        shooterRpmUpWasPressed = shooterRpmUpPressed;

        boolean shooterRpmDownPressed = RobotConstants.SHOOTER_RPM_DOWN_BUTTON.test(driverGamepad);
        if (shooterRpmDownPressed && !shooterRpmDownWasPressed) {
            shooter.adjustTargetRpm(-RobotConstants.SHOOTER_RPM_ADJUST_STEP);
        }
        shooterRpmDownWasPressed = shooterRpmDownPressed;

        // Manual driving would replace the routine's path, and the buttons would stop its shooting
        if (isAutoShooting()) {
            return;
        }

        drivetrain.driveFieldCentric(
                applyDeadband(RobotConstants.DRIVE_FORWARD_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_FORWARD_AXIS_SIGN),
                applyDeadband(RobotConstants.DRIVE_STRAFE_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_STRAFE_AXIS_SIGN),
                applyDeadband(RobotConstants.DRIVE_TURN_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_TURN_AXIS_SIGN)
        );

        // Intake is checked first, so if both buttons go down in the same loop, intake wins
        if (RobotConstants.INTAKE_BUTTON.test(driverGamepad)) {
            requestIntake();
        } else {
            stopIntake();
        }

        if (RobotConstants.SHOOT_BUTTON.test(driverGamepad)) {
            requestShoot();
        } else {
            stopShoot();
        }
    }

    /**
     * The robot's starting pose (bottom right corner, facing forward), from RobotConstants.
     *
     * @return the start pose, with x and y in inches and heading in radians
     */
    public static Pose startPose() {
        return new Pose(RobotConstants.START_POSE_X_INCHES, RobotConstants.START_POSE_Y_INCHES,
                Math.toRadians(RobotConstants.START_POSE_HEADING_DEGREES));
    }

    /**
     * The pose the localizer is set to when ZERO_POSE_BUTTON is pressed (bottom left corner), from RobotConstants.
     *
     * @return the zero pose, with x and y in inches and heading in radians
     */
    public static Pose zeroPose() {
        return new Pose(RobotConstants.ZERO_POSE_X_INCHES, RobotConstants.ZERO_POSE_Y_INCHES,
                Math.toRadians(RobotConstants.ZERO_POSE_HEADING_DEGREES));
    }

    /**
     * Sets where the auto-shoot routine drives to and how. Call once from the OpMode's init() before the
     * auto-shoot button is used; until then the button does nothing.
     *
     * @param shootingPose           the pose to shoot from, with x and y in inches and heading in radians
     * @param wallClearanceYInches   the y the robot drives to first, so it moves along x away from the wall
     * @param shootDurationSeconds   how long to shoot once at the shooting pose
     */
    public void configureAutoShoot(Pose shootingPose, double wallClearanceYInches, double shootDurationSeconds) {
        autoShootShootingPose = shootingPose;
        autoShootWallClearanceYInches = wallClearanceYInches;
        autoShootDurationSeconds = shootDurationSeconds;
    }

    /**
     * Stops the intake and schedules the auto-shoot routine, starting from the robot's current pose.
     * Does nothing if configureAutoShoot() hasn't been called.
     * The OpMode must call Scheduler.execute() every loop for it to run.
     */
    public void startAutoShoot() {
        if (autoShootShootingPose == null) {
            return;
        }
        stopIntake(); // the intake and shooter never run together, and the routine ends by shooting
        autoShootRoutine = buildAutoShootRoutine(drivetrain.getPose());
        Scheduler.schedule(autoShootRoutine);
    }

    /**
     * Cancels the auto-shoot routine. The next applyDriverControls() call hands the drivetrain and
     * shooter back to the sticks and buttons, which also stops the shooter unless SHOOT_BUTTON is held.
     */
    public void cancelAutoShoot() {
        if (autoShootRoutine != null) {
            Scheduler.cancel(autoShootRoutine);
        }
    }

    /**
     * Tells whether the auto-shoot routine is running.
     *
     * @return true from the press that starts it until it finishes shooting or is cancelled
     */
    public boolean isAutoShooting() {
        return autoShootRoutine != null && Scheduler.isScheduled(autoShootRoutine);
    }

    /**
     * Builds the auto-shoot routine: drive to the wall clearance y, then to the shooting pose's x, turn in
     * place to the shooting heading, then drive to the shooting pose's y, then shoot for the configured duration.
     * Moving along x and turning only at the clearance y keeps the robot from catching the wall on the way over.
     * The first two drive legs keep the robot's heading from routine start; the last one keeps the shooting heading.
     * Every drive leg finishes only once the robot has arrived (see Drivetrain.isAtPose()).
     *
     * @param routineStartPose where the robot is when the routine starts
     * @return the routine, ready to schedule
     */
    private Command buildAutoShootRoutine(Pose routineStartPose) {
        Pose shootingPose = autoShootShootingPose;
        Pose wallClearancePose = routineStartPose.withY(autoShootWallClearanceYInches);
        // Shooting x, still at the clearance y and the routine's starting heading
        Pose shootingXAtClearancePose = wallClearancePose.withX(shootingPose.x());
        // Same spot, turned to the shooting heading
        Pose shootingXAtClearanceTurnedPose = shootingXAtClearancePose.withHeading(shootingPose.heading());

        return sequential(
                driveStraightCommand(routineStartPose, wallClearancePose),
                driveStraightCommand(wallClearancePose, shootingXAtClearancePose),
                drivetrain.holdPoseCommand(shootingXAtClearanceTurnedPose), // turn in place to the shooting heading
                driveStraightCommand(shootingXAtClearanceTurnedPose, shootingPose),
                // Same as holding SHOOT_BUTTON: spins the wheel, and the indexer feeds once it's up to speed
                instant(shooter::requestShooting),
                waitMs(autoShootDurationSeconds * 1000.0),
                instant(shooter::requestIdle)
        );
    }

    /**
     * Builds a command that drives straight from one pose to another without turning. If the two poses
     * are already within the drive tolerance (e.g. the robot is already at that x), it just holds the end
     * pose instead, since a path with (almost) no length gives the follower no direction to drive in.
     *
     * @param legStartPose where the leg starts; must have the same heading as legEndPose
     * @param legEndPose   where the leg ends
     * @return a command that finishes once the robot is at legEndPose
     */
    private Command driveStraightCommand(Pose legStartPose, Pose legEndPose) {
        if (legStartPose.distance(legEndPose) < RobotConstants.DRIVE_POSITION_TOLERANCE_INCHES) {
            return drivetrain.holdPoseCommand(legEndPose);
        }
        return drivetrain.followPathCommand(line(legStartPose, legEndPose).linear(legStartPose, legEndPose));
    }

    /**
     * Starts intaking, unless the shooter is running. In that case the request is rejected
     * outright, not queued, so the intake will not start later by itself.
     */
    public void requestIntake() {
        if (!shooter.isActive()) {
            intake.requestIntaking();
        }
    }

    /**
     * Stops the intake.
     */
    public void stopIntake() {
        intake.requestIdle();
    }

    /**
     * Starts shooting, unless the intake is running. In that case the request is rejected
     * outright, not queued, so the shooter will not start later by itself.
     */
    public void requestShoot() {
        if (!intake.isActive()) {
            shooter.requestShooting();
        }
    }

    /**
     * Stops the shooter.
     */
    public void stopShoot() {
        shooter.requestIdle();
    }

    /**
     * The robot's current super state, found by matching the subsystems' actual states against
     * each RobotSuperState's definition of what they should be. Falls back to IDLE if the current
     * combination of subsystem states doesn't match any defined super state.
     *
     * @return the matching super state
     */
    public RobotSuperState getSuperState() {
        for (RobotSuperState superState : RobotSuperState.values()) {
            if (superState.intakeState == intake.getState() && superState.shooterState == shooter.getState()) {
                return superState;
            }
        }
        return RobotSuperState.IDLE;
    }

    /**
     * Zeroes a stick value that is small enough to be drift rather than real input.
     *
     * @param stickValue the stick value, from -1 to 1
     * @return 0 if the value is within the deadband, otherwise the value unchanged
     */
    private double applyDeadband(double stickValue) {
        return Math.abs(stickValue) <= RobotConstants.STICK_DEADBAND ? 0.0 : stickValue;
    }
}
