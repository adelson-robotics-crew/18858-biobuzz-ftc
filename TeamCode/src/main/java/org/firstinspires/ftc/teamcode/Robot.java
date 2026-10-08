package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTarget;
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

    // The alliance of the last autonomous that ran. Static so it survives from the autonomous into the TeleOp (and
    // into a TeleOp restarted mid-match) while the app keeps running; a power cycle or app restart clears it.
    // Picking the red or blue auto is how the robot knows its side; it can't sense it
    private static Alliance savedAlliance = null;

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;

    // Whether each shooter-RPM button was held last loop, so a hold counts as one press instead of one per loop
    private boolean shooterRpmUpWasPressed = false;
    private boolean shooterRpmDownWasPressed = false;
    private boolean resetHeadingWasPressed = false;

    // Whether RESET_HEADING_BUTTON resets the whole pose to (0, 0, 0 deg) instead of only the heading. Only the
    // practice TeleOp turns this on (see setResetButtonResetsPosition())
    private boolean resetButtonResetsPosition = false;

    // Wheel speed for a manual shot (SHOOT_BUTTON). Fixed at RobotConstants.SHOOTER_TARGET_RPM unless the bumpers
    // adjust it (only while RobotConstants.SHOOTER_RPM_ADJUST_ENABLED). Kept here rather than in the shooter so an
    // aimed shot, which sets the shooter to the shot table's RPM, can't change what the next manual shot uses
    private double manualShotRpm = RobotConstants.SHOOTER_TARGET_RPM;

    // The HIVE cells the aim buttons shoot into, as the driver sees them; set by the TeleOp for its alliance
    // (see setShotTargets()). Null until set, and an aim button with no cell does nothing
    private ShotTarget leftShotTarget = null;
    private ShotTarget rightShotTarget = null;

    // The cell being aimed at and the shot worked out from the robot's pose this loop, while an aim button is
    // held; both null while neither is
    private ShotTarget currentShotTarget = null;
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
     * Records which alliance the autonomous is running for, so the TeleOp that follows aims at the same hive.
     * The autonomous calls this in init().
     *
     * @param alliance the autonomous's alliance
     */
    public static void saveAlliance(Alliance alliance) {
        savedAlliance = alliance;
    }

    /**
     * The alliance of the last autonomous that ran.
     *
     * @return the saved alliance, or null if no autonomous has run since the app started (e.g. after a power cycle)
     */
    public static Alliance getSavedAlliance() {
        return savedAlliance;
    }

    /**
     * Sets which HIVE cells AIM_LEFT_CELL_BUTTON and AIM_RIGHT_CELL_BUTTON shoot into. The TeleOp calls this once
     * in init() with its alliance's cells.
     *
     * @param leftCell  the cell on the driver's left
     * @param rightCell the cell on the driver's right
     */
    public void setShotTargets(ShotTarget leftCell, ShotTarget rightCell) {
        leftShotTarget = leftCell;
        rightShotTarget = rightCell;
    }

    /**
     * Sets whether RESET_HEADING_BUTTON resets the whole pose to (0, 0, 0 deg) or only the heading.
     *
     * @param resetsPosition true to reset x, y, and heading; false (the default) to reset only the heading
     */
    public void setResetButtonResetsPosition(boolean resetsPosition) {
        resetButtonResetsPosition = resetsPosition;
    }

    /**
     * Updates every subsystem. Call this exactly once per OpMode loop.
     */
    public void update() {
        drivetrain.update();
        intake.update();
        shooter.update(); // the indexer holds its rest angle by itself whenever it isn't feeding
    }

    /**
     * Reads the driver gamepad through the mapping in RobotConstants and sends the resulting
     * commands to the subsystems. Call once per TeleOp loop, before update().
     * Shooting, in priority order:
     *   1. SHOOT_BUTTON held: manual shot. The driver turns with the turn stick and the wheel spins at the fixed
     *      manual RPM; X/B are ignored while it's held.
     *   2. AIM_LEFT_CELL_BUTTON or AIM_RIGHT_CELL_BUTTON held: the robot aims and shoots at that cell (see
     *      applyAimAndShoot()): it turns itself to the shot heading and the wheel follows the shot table.
     *   3. Neither: normal driving, shooter off.
     * Nothing latches: as soon as SHOOT_BUTTON is released, holding X or B aims again.
     *
     * @param driverGamepad the gamepad that drives the robot (gamepad1 in the OpMode)
     */
    public void applyDriverControls(Gamepad driverGamepad) {
        // Reset, once per press: the robot's current facing becomes 0 deg (+x), and in the practice TeleOp its
        // position becomes (0, 0) too
        boolean resetHeadingPressed = RobotConstants.RESET_HEADING_BUTTON.test(driverGamepad);
        if (resetHeadingPressed && !resetHeadingWasPressed) {
            if (resetButtonResetsPosition) {
                drivetrain.setPose(new Pose(0.0, 0.0, 0.0));
            } else {
                drivetrain.resetHeading();
            }
        }
        resetHeadingWasPressed = resetHeadingPressed;

        // Bump the manual shot RPM once per press, only on the loop the button goes down, and only while RPM
        // adjusting is turned on (it's off for matches). Aimed shots use the shot table, so this doesn't affect them.
        if (RobotConstants.SHOOTER_RPM_ADJUST_ENABLED) {
            boolean shooterRpmUpPressed = RobotConstants.SHOOTER_RPM_UP_BUTTON.test(driverGamepad);
            if (shooterRpmUpPressed && !shooterRpmUpWasPressed) {
                adjustManualShotRpm(RobotConstants.SHOOTER_RPM_ADJUST_STEP);
            }
            shooterRpmUpWasPressed = shooterRpmUpPressed;

            boolean shooterRpmDownPressed = RobotConstants.SHOOTER_RPM_DOWN_BUTTON.test(driverGamepad);
            if (shooterRpmDownPressed && !shooterRpmDownWasPressed) {
                adjustManualShotRpm(-RobotConstants.SHOOTER_RPM_ADJUST_STEP);
            }
            shooterRpmDownWasPressed = shooterRpmDownPressed;
        }

        double forward = applyDeadband(RobotConstants.DRIVE_FORWARD_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_FORWARD_AXIS_SIGN);
        double strafe = applyDeadband(RobotConstants.DRIVE_STRAFE_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_STRAFE_AXIS_SIGN);
        double turn = applyDeadband(RobotConstants.DRIVE_TURN_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_TURN_AXIS_SIGN);

        // Intake is checked first, so if intake and a shoot button go down in the same loop, intake wins
        if (RobotConstants.INTAKE_BUTTON.test(driverGamepad)) {
            requestIntake();
        } else {
            stopIntake();
        }

        boolean manualShootHeld = RobotConstants.SHOOT_BUTTON.test(driverGamepad);

        // Aim and shoot, unless the manual shot button overrides it. Left is checked first, so if both aim buttons
        // are held, the left cell wins
        if (!manualShootHeld) {
            if (RobotConstants.AIM_LEFT_CELL_BUTTON.test(driverGamepad) && leftShotTarget != null) {
                applyAimAndShoot(leftShotTarget, forward, strafe);
                return;
            }
            if (RobotConstants.AIM_RIGHT_CELL_BUTTON.test(driverGamepad) && rightShotTarget != null) {
                applyAimAndShoot(rightShotTarget, forward, strafe);
                return;
            }
        }

        // Manual shot or plain driving: the driver turns, and the wheel uses the fixed manual RPM
        currentShotTarget = null;
        currentShotSolution = null;
        drivetrain.driveFieldCentric(forward, strafe, turn);
        shooter.setFeedAllowed(true); // manual shooting feeds whenever the wheel is at speed
        // Manual shots always use the fixed manual RPM, never the shot table: an aimed shot may have left the
        // shooter at the table's RPM, so set it back every loop
        shooter.setTargetRpm(manualShotRpm);
        if (manualShootHeld) {
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
     * @param target  the HIVE cell to shoot into
     * @param forward the driver's forward power, from -1 to 1
     * @param strafe  the driver's sideways power, from -1 to 1
     */
    private void applyAimAndShoot(ShotTarget target, double forward, double strafe) {
        currentShotTarget = target;
        currentShotSolution = ShotSolver.solve(drivetrain.getPose(), target);
        drivetrain.driveFieldCentricWithHeading(forward, strafe, currentShotSolution.targetHeadingRadians);
        shooter.setTargetRpm(currentShotSolution.targetRpm);
        shooter.setFeedAllowed(currentShotSolution.inRange && isAimed());
        requestShoot(); // still rejected while the intake runs
    }

    /**
     * Changes the manual shot RPM by the given amount, kept between RobotConstants.SHOOTER_MIN_TARGET_RPM and
     * RobotConstants.SHOOTER_MAX_TARGET_RPM.
     *
     * @param rpmChange how much to add; negative to lower it
     */
    private void adjustManualShotRpm(double rpmChange) {
        manualShotRpm = Math.max(RobotConstants.SHOOTER_MIN_TARGET_RPM,
                Math.min(RobotConstants.SHOOTER_MAX_TARGET_RPM, manualShotRpm + rpmChange));
    }

    /**
     * The wheel speed a manual shot (SHOOT_BUTTON) uses.
     *
     * @return the manual shot RPM
     */
    public double getManualShotRpm() {
        return manualShotRpm;
    }

    /**
     * The shot worked out this loop while an aim button is held.
     *
     * @return the current shot solution, or null while not aiming
     */
    public ShotSolution getShotSolution() {
        return currentShotSolution;
    }

    /**
     * The HIVE cell being aimed at this loop.
     *
     * @return the current cell, or null while not aiming
     */
    public ShotTarget getShotTarget() {
        return currentShotTarget;
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
