package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTarget;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Owns every subsystem and is the single place hardware gets initialized. It also holds the logic
 * that ties the subsystems together: reading the gamepad through the mapping in RobotConstants,
 * deciding which mechanisms may run together (the intake and shooter never do), and when the indexer
 * feeds (only once the shooter's wheel is at speed; see update()). Subsystems never call each other.
 * OpModes create one Robot in init() and talk to the subsystems through it;
 * they never touch hardware objects directly.
 */
public class Robot {

    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Stick values with a magnitude at or below this are treated as 0.
    // 0.0 means no deadband, which is how the TeleOp behaved before this constant existed.
    private static final double STICK_DEADBAND = 0.0;

    // How much one press of SHOOTER_RPM_UP_BUTTON / SHOOTER_RPM_DOWN_BUTTON changes the target RPM.
    private static final double SHOOTER_RPM_ADJUST_STEP = 50.0;

    // The indexer only feeds once the robot's heading is within this many degrees of the shooting heading.
    // Every measured scoring shot was within 5 deg of the computed heading.
    private static final double HEADING_TOLERANCE_DEG = 3.0;

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
        OUTTAKING("Outtaking", Intake.State.OUTTAKING, Shooter.State.IDLE),
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
    public final Indexer indexer;

    // The battery voltage as the hub measures it. Not part of any subsystem: it's read for logging, since the
    // shooter's behavior has been seen to change as the battery runs down
    private final VoltageSensor batteryVoltageSensor;

    // Whether the indexer may feed once the shooter's wheel is at speed (see update()). Set every loop by whatever
    // is shooting: while aiming, only once the position is valid and the robot is aimed; otherwise true
    private boolean feedAllowed = true;
    // Whether the shooter was running last loop, so update() can tell the loop it starts (to correct the indexer)
    private boolean shooterWasActive = false;

    // Whether each shooter-RPM button was held last loop, so a hold counts as one press instead of one per loop
    private boolean shooterRpmUpWasPressed = false;
    private boolean shooterRpmDownWasPressed = false;
    private boolean resetHeadingWasPressed = false;

    // Whether RESET_HEADING_BUTTON resets the whole pose to (0, 0, 0 deg) instead of only the heading. Only the
    // practice TeleOp turns this on (see setResetButtonResetsPosition())
    private boolean resetButtonResetsPosition = false;

    // Whether the driver stands on the blue side of the field (opposite the red driver). Pedro's +x is "away from
    // the driver" only for the red driver; for the blue driver it's toward them, so the sticks are turned 180 deg
    // (see setDriverOnBlueSide()). Only the driver's input changes: the odometry, autos, and aiming are untouched
    private boolean driverOnBlueSide = false;

    // Wheel speed for a manual shot (SHOOT_BUTTON). Fixed at RobotConstants.SHOOTER_TARGET_RPM unless the bumpers
    // adjust it (only while RobotConstants.SHOOTER_RPM_ADJUST_ENABLED). Kept here rather than in the shooter so an
    // aimed shot, which sets the shooter to the shot table's RPM, can't change what the next manual shot uses
    private double manualShotRpm = RobotConstants.SHOOTER_TARGET_RPM;

    // The alliance's two HIVE cells, which AIM_AND_SHOOT_BUTTON picks between; set by the TeleOp for its alliance
    // (see setShotTargets()). Null until set, and the aim button does nothing without them
    private ShotTarget firstShotTarget = null;
    private ShotTarget secondShotTarget = null;

    // The cell being aimed at and the shot worked out from the robot's pose this loop, while the aim button is
    // held; both null while it isn't. The cell is picked once per press, so it doesn't switch mid-shot if the robot
    // drifts across the center line
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
        indexer = new Indexer(hardwareMap);
        // The hub's voltage sensor; there's one per hub, and any of them reads the same battery
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * The battery voltage right now, as the hub measures it. A hub read (a couple of milliseconds), so read it once
     * per loop at most.
     *
     * @return the battery voltage, in volts
     */
    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
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
     * Sets the two HIVE cells AIM_AND_SHOOT_BUTTON picks between. The TeleOp calls this once in init() with its
     * alliance's cells; their order doesn't matter.
     *
     * @param firstCell  one of the alliance's cells
     * @param secondCell the alliance's other cell
     */
    public void setShotTargets(ShotTarget firstCell, ShotTarget secondCell) {
        firstShotTarget = firstCell;
        secondShotTarget = secondCell;
    }

    /**
     * Picks the cell on the robot's half of the field: the one whose y is nearer the robot's y. The two cells
     * mirror each other over the field's horizontal center line, so this is the same as asking whether the robot is
     * above or below that line, and it still works in a TeleOp whose coordinates are shifted (cells shift too).
     * Red below the line gets its right (lower) cell and above it its left (upper) cell; for blue, below gets the
     * left (lower) cell and above the right (upper) one.
     *
     * @return the cell to aim at from where the robot is now
     */
    private ShotTarget chooseShotTarget() {
        double robotYInches = drivetrain.getPose().y();
        double distanceToFirstCell = Math.abs(robotYInches - firstShotTarget.yInches);
        double distanceToSecondCell = Math.abs(robotYInches - secondShotTarget.yInches);
        return distanceToFirstCell <= distanceToSecondCell ? firstShotTarget : secondShotTarget;
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
     * Sets which side of the field the driver stands on, so field-centric "forward" is away from them. On the blue
     * side the stick's forward and strafe are turned 180 deg, and RESET_HEADING_BUTTON makes the robot's current
     * facing 180 deg (Pedro's "away from the blue driver") instead of 0 deg.
     *
     * @param onBlueSide true when the driver stands on the blue side; false (the default) for the red side
     */
    public void setDriverOnBlueSide(boolean onBlueSide) {
        driverOnBlueSide = onBlueSide;
    }

    /**
     * Tells which side of the field the driver controls are set up for.
     *
     * @return true if the controls are from the blue driver's side, false if from the red driver's side
     */
    public boolean isDriverOnBlueSide() {
        return driverOnBlueSide;
    }

    /**
     * Updates every subsystem. Call this exactly once per OpMode loop.
     */
    public void update() {
        drivetrain.update();
        intake.update();
        shooter.update(); // also updates whether the wheel is at speed, which the indexer decision below reads

        // Feed only while the shooter is running with its wheel at speed and feeding is allowed, so no ball is
        // fed into a slow (or overshooting) wheel. Otherwise stop feeding: the indexer's PID corrects it back to
        // rest for a short window, then it cuts power until the next shot
        boolean shooterJustStarted = shooter.isActive() && !shooterWasActive;
        shooterWasActive = shooter.isActive();
        if (shooter.isActive() && shooter.isAtSpeed() && feedAllowed) {
            indexer.requestFeeding();
        } else {
            indexer.stopFeeding();
            if (shooterJustStarted) {
                // A shot (A or the aim trigger) just started and the wheel is still spinning up: correct the indexer
                // back to rest now, since it may have been knocked off while unpowered, before it starts feeding
                indexer.prepareForShot();
            }
        }
        indexer.update();
    }

    /**
     * Sets whether the indexer may feed. Even when allowed, it only feeds while the shooter runs with its wheel at
     * speed (see update()). Whatever is shooting sets this every loop: aim-and-shoot only once the position is
     * valid and the robot is aimed, a manual or fixed-RPM shot always.
     *
     * @param allowed true to let the indexer feed once the wheel is at speed; false to hold balls back
     */
    public void setFeedAllowed(boolean allowed) {
        feedAllowed = allowed;
    }

    /**
     * Reads the driver gamepad through the mapping in RobotConstants and sends the resulting
     * commands to the subsystems. Call once per TeleOp loop, before update().
     * Shooting, in priority order:
     *   1. SHOOT_BUTTON held: manual shot. The driver turns with the turn stick and the wheel spins at the fixed
     *      manual RPM; the right trigger is ignored while it's held.
     *   2. AIM_AND_SHOOT_BUTTON held: the robot aims and shoots at the alliance cell on its half of the field (see
     *      chooseShotTarget() and applyAimAndShoot()): it turns itself to the shot heading and the wheel follows
     *      the shot table.
     *   3. Neither: normal driving, shooter off.
     * Nothing latches: as soon as SHOOT_BUTTON is released, holding the right trigger aims again.
     *
     * @param driverGamepad the gamepad that drives the robot (gamepad1 in the OpMode)
     */
    public void applyDriverControls(Gamepad driverGamepad) {
        // Reset, once per press: the robot's current facing becomes "away from the driver" (0 deg from the red side,
        // 180 deg from the blue side), and in the practice TeleOp its position becomes (0, 0) too
        boolean resetHeadingPressed = RobotConstants.RESET_HEADING_BUTTON.test(driverGamepad);
        if (resetHeadingPressed && !resetHeadingWasPressed) {
            if (resetButtonResetsPosition) {
                drivetrain.setPose(new Pose(0.0, 0.0, 0.0));
            } else {
                drivetrain.resetHeading(driverOnBlueSide ? Math.PI : 0.0);
            }
        }
        resetHeadingWasPressed = resetHeadingPressed;

        // Bump the manual shot RPM once per press, only on the loop the button goes down, and only while RPM
        // adjusting is turned on (it's off for matches). Aimed shots use the shot table, so this doesn't affect them.
        if (RobotConstants.SHOOTER_RPM_ADJUST_ENABLED) {
            boolean shooterRpmUpPressed = RobotConstants.SHOOTER_RPM_UP_BUTTON.test(driverGamepad);
            if (shooterRpmUpPressed && !shooterRpmUpWasPressed) {
                adjustManualShotRpm(SHOOTER_RPM_ADJUST_STEP);
            }
            shooterRpmUpWasPressed = shooterRpmUpPressed;

            boolean shooterRpmDownPressed = RobotConstants.SHOOTER_RPM_DOWN_BUTTON.test(driverGamepad);
            if (shooterRpmDownPressed && !shooterRpmDownWasPressed) {
                adjustManualShotRpm(-SHOOTER_RPM_ADJUST_STEP);
            }
            shooterRpmDownWasPressed = shooterRpmDownPressed;
        }

        double forward = applyDeadband(RobotConstants.DRIVE_FORWARD_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_FORWARD_AXIS_SIGN);
        double strafe = applyDeadband(RobotConstants.DRIVE_STRAFE_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_STRAFE_AXIS_SIGN);
        double turn = applyDeadband(RobotConstants.DRIVE_TURN_AXIS.applyAsDouble(driverGamepad) * RobotConstants.DRIVE_TURN_AXIS_SIGN);
        if (driverOnBlueSide) {
            // The blue driver faces the opposite way down the field, so turn the stick 180 deg (flip forward and
            // strafe). Turning is clockwise/counterclockwise from either side, so it stays as is
            forward = -forward;
            strafe = -strafe;
        }

        // Intake is checked first, so if intake and a shoot button go down in the same loop, intake wins.
        // Intake also wins over outtake when both are held
        if (RobotConstants.INTAKE_BUTTON.test(driverGamepad)) {
            requestIntake();
        } else if (RobotConstants.OUTTAKE_BUTTON.test(driverGamepad)) {
            requestOuttake();
        } else {
            stopIntake();
        }

        boolean manualShootHeld = RobotConstants.SHOOT_BUTTON.test(driverGamepad);

        // Aim and shoot, unless the manual shot button overrides it
        boolean aimHeld = RobotConstants.AIM_AND_SHOOT_BUTTON.test(driverGamepad);
        if (!manualShootHeld && aimHeld && firstShotTarget != null && secondShotTarget != null) {
            // Pick the cell when aiming starts and keep it for the rest of the press
            ShotTarget target = currentShotTarget != null ? currentShotTarget : chooseShotTarget();
            applyAimAndShoot(target, forward, strafe);
            return;
        }

        // Manual shot or plain driving: the driver turns, and the wheel uses the fixed manual RPM
        currentShotTarget = null;
        currentShotSolution = null;
        drivetrain.driveFieldCentric(forward, strafe, turn);
        setFeedAllowed(true); // manual shooting feeds whenever the wheel is at speed
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
     * One loop of aim-and-shoot without a driver, for an autonomous: the same as holding AIM_AND_SHOOT_BUTTON with
     * the sticks centered. Picks the alliance cell on the robot's half of the field on the first call and keeps it
     * until stopAimAndShoot(). Call every loop while shooting; does nothing until setShotTargets() has been called.
     */
    public void aimAndShootInPlace() {
        if (firstShotTarget == null || secondShotTarget == null) {
            return;
        }
        ShotTarget target = currentShotTarget != null ? currentShotTarget : chooseShotTarget();
        applyAimAndShoot(target, 0.0, 0.0); // no translation: turn in place
    }

    /**
     * Ends an aimAndShootInPlace() run: stops the shooter and forgets the cell, so the next aim picks one again.
     */
    public void stopAimAndShoot() {
        currentShotTarget = null;
        currentShotSolution = null;
        stopShoot();
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
        setFeedAllowed(currentShotSolution.inRange && isAimed());
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
     * The shot as it stands from where the robot is now, whether or not aim-and-shoot is held: the shot being taken
     * while aiming, otherwise the shot at the cell the aim button would pick from here. Lets the driver see "IN
     * RANGE" / "TOO FAR" etc. before pulling the trigger. Doesn't move anything.
     *
     * @return the shot solution, or null if the TeleOp hasn't set the alliance's cells
     */
    public ShotSolution getShotSolutionFromHere() {
        if (currentShotSolution != null) {
            return currentShotSolution;
        }
        if (firstShotTarget == null || secondShotTarget == null) {
            return null;
        }
        return ShotSolver.solve(drivetrain.getPose(), chooseShotTarget());
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
     * Tells whether the robot is facing the shooting heading, within HEADING_TOLERANCE_DEG.
     *
     * @return true while aiming and within the heading tolerance
     */
    public boolean isAimed() {
        return currentShotSolution != null
                && Math.abs(getAimHeadingErrorDeg()) <= HEADING_TOLERANCE_DEG;
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
     * Starts outtaking (the intake run backwards), unless the shooter is running. In that case the request is
     * rejected outright, not queued, so the outtake will not start later by itself.
     */
    public void requestOuttake() {
        if (!shooter.isActive()) {
            intake.requestOuttaking();
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
        return Math.abs(stickValue) <= STICK_DEADBAND ? 0.0 : stickValue;
    }
}
