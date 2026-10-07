package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolver;
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
    private boolean zeroPoseWasPressed = false;

    // The shot worked out from the robot's pose this loop while AIM_SHOOT_BUTTON is held, or null while it isn't
    private ShotSolution currentShotSolution = null;

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
        // The indexer belongs to the shooter, so the intake can't run it; instead, run it backward while intaking
        // to keep incoming balls off the shooter wheel
        shooter.setIndexerReverse(intake.isActive());
        shooter.update();
    }

    /**
     * Reads the driver gamepad through the mapping in RobotConstants and sends the resulting
     * commands to the subsystems. Call once per TeleOp loop, before update().
     * While AIM_SHOOT_BUTTON is held, the robot aims and shoots (see applyAimAndShoot()) instead of
     * turning with the turn stick and shooting on SHOOT_BUTTON.
     *
     * @param driverGamepad the gamepad that drives the robot (gamepad1 in the OpMode)
     */
    public void applyDriverControls(Gamepad driverGamepad) {
        // Reset the localizer to the zero pose once per press
        boolean zeroPosePressed = RobotConstants.ZERO_POSE_BUTTON.test(driverGamepad);
        if (zeroPosePressed && !zeroPoseWasPressed) {
            drivetrain.setPose(zeroPose());
        }
        zeroPoseWasPressed = zeroPosePressed;

        // Bump the shooter target RPM once per press, only on the loop the button goes down.
        // While aiming, the shot table sets the target every loop, so a bump there is overwritten right away.
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

        double forward = applyDeadband(RobotConstants.DRIVE_FORWARD_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_FORWARD_AXIS_SIGN);
        double strafe = applyDeadband(RobotConstants.DRIVE_STRAFE_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_STRAFE_AXIS_SIGN);
        double turn = applyDeadband(RobotConstants.DRIVE_TURN_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_TURN_AXIS_SIGN);

        // Intake is checked first, so if intake and a shoot button go down in the same loop, intake wins
        if (RobotConstants.INTAKE_BUTTON.test(driverGamepad)) {
            requestIntake();
        } else {
            stopIntake();
        }

        if (RobotConstants.AIM_SHOOT_BUTTON.test(driverGamepad)) {
            applyAimAndShoot(forward, strafe);
            return;
        }

        currentShotSolution = null;
        drivetrain.driveFieldCentric(forward, strafe, turn);
        shooter.setFeedAllowed(true); // manual shooting feeds whenever the wheel is at speed, as before
        if (RobotConstants.SHOOT_BUTTON.test(driverGamepad)) {
            requestShoot();
        } else {
            stopShoot();
        }
    }

    /**
     * One loop of aim-and-shoot: solves the shot from the robot's pose, turns the robot to the shooting heading
     * while the driver keeps translation, holds the shooter at the shot table's RPM for the current distance,
     * and lets the indexer feed only while the position is valid and the robot is aimed. The wheel keeps spinning
     * at the table RPM even while the position isn't valid, so it's already at speed once the robot lines up.
     *
     * @param forward the driver's forward power, from -1 to 1
     * @param strafe  the driver's sideways power, from -1 to 1
     */
    private void applyAimAndShoot(double forward, double strafe) {
        currentShotSolution = ShotSolver.solve(drivetrain.getPose());
        drivetrain.driveFieldCentricWithHeading(forward, strafe, currentShotSolution.targetHeadingRadians);
        shooter.setTargetRpm(currentShotSolution.targetRpm);
        shooter.setFeedAllowed(currentShotSolution.inRange && isAimed());
        requestShoot(); // still rejected while the intake runs
    }

    /**
     * The shot worked out this loop while AIM_SHOOT_BUTTON is held.
     *
     * @return the current shot solution, or null while not aiming
     */
    public ShotSolution getShotSolution() {
        return currentShotSolution;
    }

    /**
     * How far the robot's heading is from the shooting heading.
     *
     * @return heading error in degrees (target minus current, wrapped to [-180, 180)), or 0 while not aiming
     */
    public double getAimHeadingErrorDeg() {
        if (currentShotSolution == null) {
            return 0.0;
        }
        return drivetrain.headingErrorDeg(currentShotSolution.targetHeadingRadians);
    }

    /**
     * Tells whether the robot is facing the shooting heading, within RobotConstants.HEADING_TOLERANCE_DEG.
     *
     * @return true while aiming and within the heading tolerance
     */
    public boolean isAimed() {
        return currentShotSolution != null
                && Math.abs(getAimHeadingErrorDeg()) <= RobotConstants.HEADING_TOLERANCE_DEG;
    }

    /**
     * Tells whether everything needed to feed a shot is true: valid position, aimed, and the wheel at speed.
     *
     * @return true while aiming with all three conditions met
     */
    public boolean isReadyToShoot() {
        return currentShotSolution != null && currentShotSolution.inRange && isAimed() && shooter.isAtSpeed();
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
