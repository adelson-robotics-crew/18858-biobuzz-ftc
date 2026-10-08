package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * The shooter: a DC motor that spins the shooting wheel, plus the indexer that feeds balls into it: an Axon servo
 * in continuous mode, whose feedback wire (an analog input) reports its angle within one turn. Only Robot decides when the shooter may run (see Robot.requestShoot()).
 * While shooting, the wheel is held at a fixed RPM with closed-loop velocity control, so its speed
 * doesn't drop as the battery drains. The indexer is not requested separately: it feeds automatically
 * while the wheel is being driven (SHOOTING), the wheel is at speed, and Robot allows feeding (setFeedAllowed()).
 * "At speed" has hysteresis: the wheel must get within RobotConstants.INDEXER_START_FEED_RPM_TOLERANCE of the
 * target to count, and stops counting once it's more than RobotConstants.INDEXER_STOP_FEED_RPM_TOLERANCE away.
 * So a ball is never fed into a wheel that is too slow (still spinning up, coasting down) or too fast
 * (overshooting after a spin-up or a target change), and the indexer doesn't flicker when a ball briefly drags
 * the wheel down. Whenever the indexer isn't feeding, it turns to the nearest rest angle and holds there, so balls
 * can't feed through (see IndexerRest). Rest angles are every RobotConstants.INDEXER_REST_SPACING_DEG from where the
 * indexer was at INIT.
 * The target RPM starts at RobotConstants.SHOOTER_TARGET_RPM and can be adjusted at runtime with adjustTargetRpm()
 * (so the right shooting speed can be found on the field) or set outright with setTargetRpm().
 */
public class Shooter {
    public enum State {
        IDLE,
        SHOOTING
    }

    private final DcMotorEx shooterMotor; // DcMotorEx (not DcMotor) so the encoder velocity can be read and set
    private final CRServo indexerServo;
    private final AnalogInput indexerEncoder; // the Axon's position feedback: 0 V to max voltage = one full turn
    private final double indexerZeroAngleDegrees; // the encoder's angle at INIT; rest angles are measured from here
    private State state = State.IDLE; // flywheel state
    private double peakShooterRpm = 0.0; // highest RPM magnitude seen since init, for finding the motor's max speed
    private double targetRpm = RobotConstants.SHOOTER_TARGET_RPM; // wheel speed held while shooting; adjustable at runtime
    private boolean indexerFeeding = false; // whether the indexer fed this loop (otherwise it was holding rest)
    private boolean feedAllowed = true; // whether Robot allows the indexer to feed (e.g. only once aimed); set every loop
    private boolean wheelAtSpeed = false; // the hysteresis latch: true once within the start tolerance, until past the stop tolerance
    /**
     * Gets the shooter hardware from the hardware map.
     *
     * @param hardwareMap the hardware map from the running OpMode
     */
    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_MOTOR_NAME);
        indexerServo = hardwareMap.get(CRServo.class, RobotConstants.INDEXER_SERVO_NAME);
        indexerEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.INDEXER_ENCODER_NAME);
        // Wherever the indexer is at INIT is 0 deg, so it should be placed in a rest position before INIT
        indexerZeroAngleDegrees = readIndexerEncoderDegrees();
        // Lets the motor controller use the encoder to hold the speed passed to setVelocity()
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (RobotConstants.SHOOTER_USE_CUSTOM_VELOCITY_PIDF) {
            // Replaces the controller's default velocity gains for RUN_USING_ENCODER (the mode setVelocity() uses)
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    RobotConstants.SHOOTER_VELOCITY_P,
                    RobotConstants.SHOOTER_VELOCITY_I,
                    RobotConstants.SHOOTER_VELOCITY_D,
                    RobotConstants.SHOOTER_VELOCITY_F));
        }
    }

    /**
     * Applies power (or target velocity) for the current state. Call once per loop.
     */
    public void update() {
        peakShooterRpm = Math.max(peakShooterRpm, Math.abs(getShooterRpm()));

        switch (state) {
            case IDLE:
                shooterMotor.setPower(0.0);
                break;
            case SHOOTING:
                shooterMotor.setVelocity(rpmToTicksPerSecond(targetRpm));
                break;
        }

        // The wheel is only ever at speed while it's being driven (not coasting). Otherwise it latches at speed once
        // it's within the start tolerance of the target (above or below), and unlatches once past the stop tolerance
        double rpmError = Math.abs(getShooterRpm() - targetRpm);
        if (state != State.SHOOTING) {
            wheelAtSpeed = false;
        } else if (wheelAtSpeed) {
            wheelAtSpeed = rpmError <= RobotConstants.INDEXER_STOP_FEED_RPM_TOLERANCE;
        } else {
            wheelAtSpeed = rpmError <= RobotConstants.INDEXER_START_FEED_RPM_TOLERANCE;
        }

        // Feed while the wheel is at speed and Robot allows it; otherwise turn to the nearest rest angle and hold it
        indexerFeeding = wheelAtSpeed && feedAllowed;
        if (indexerFeeding) {
            indexerServo.setPower(RobotConstants.INDEXER_SERVO_POWER);
        } else {
            indexerServo.setPower(IndexerRest.powerToRest(getIndexerErrorToRestDegrees()));
        }
    }

    /**
     * Reads the indexer's raw angle from the Axon's feedback voltage.
     *
     * @return the encoder angle, from 0 to 360 deg
     */
    private double readIndexerEncoderDegrees() {
        // The feedback voltage sweeps from 0 to the hub's max analog voltage over one turn
        return indexerEncoder.getVoltage() / indexerEncoder.getMaxVoltage()
                * RobotConstants.INDEXER_ENCODER_DEGREES_PER_TURN;
    }

    /**
     * The raw voltage on the indexer's feedback wire, for checking the wiring: it should sweep from about 0 V up to
     * the hub's max analog voltage (about 3.3 V) as the indexer turns once. Stuck near 0 V means no signal.
     *
     * @return the feedback voltage, in volts
     */
    public double getIndexerEncoderVoltage() {
        return indexerEncoder.getVoltage();
    }

    /**
     * The indexer's angle from where it was at INIT, in the direction positive servo power turns it.
     *
     * @return the angle, in [0, 360) deg
     */
    public double getIndexerAngleDegrees() {
        return IndexerRest.angleFromStartDegrees(readIndexerEncoderDegrees(), indexerZeroAngleDegrees,
                RobotConstants.INDEXER_ENCODER_DIRECTION);
    }

    /**
     * How far the indexer is from the nearest rest angle.
     *
     * @return the turn to the nearest rest angle, in degrees (positive = the positive-power way)
     */
    public double getIndexerErrorToRestDegrees() {
        return IndexerRest.errorToNearestRestDegrees(getIndexerAngleDegrees(), RobotConstants.INDEXER_REST_SPACING_DEG);
    }

    /**
     * Tells whether the indexer fed balls this loop, as of the last update().
     *
     * @return true while feeding; false while it's holding (or turning back to) a rest angle
     */
    public boolean isIndexerFeeding() {
        return indexerFeeding;
    }

    /**
     * Reads the shooter wheel's current speed from the motor encoder.
     *
     * @return the shooter motor's speed in revolutions per minute; negative when spinning backward
     */
    public double getShooterRpm() {
        // getVelocity() is in encoder ticks per second; * 60 gives ticks per minute, / ticks-per-rev gives RPM
        return shooterMotor.getVelocity() * 60.0 / RobotConstants.SHOOTER_ENCODER_TICKS_PER_REV;
    }

    /**
     * The speed the wheel is held at while shooting.
     *
     * @return the current target RPM
     */
    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * Changes the target RPM by the given amount, kept between RobotConstants.SHOOTER_MIN_TARGET_RPM
     * and RobotConstants.SHOOTER_MAX_TARGET_RPM. Takes effect on the next update(), even mid-shot.
     *
     * @param rpmChange how much to add to the target RPM; negative to lower it
     */
    public void adjustTargetRpm(double rpmChange) {
        targetRpm = Math.max(RobotConstants.SHOOTER_MIN_TARGET_RPM,
                Math.min(RobotConstants.SHOOTER_MAX_TARGET_RPM, targetRpm + rpmChange));
    }

    /**
     * Sets the target RPM outright, kept between RobotConstants.SHOOTER_MIN_TARGET_RPM and
     * RobotConstants.SHOOTER_MAX_TARGET_RPM. Takes effect on the next update(), even mid-shot.
     *
     * @param newTargetRpm the wheel speed to hold while shooting
     */
    public void setTargetRpm(double newTargetRpm) {
        targetRpm = Math.max(RobotConstants.SHOOTER_MIN_TARGET_RPM,
                Math.min(RobotConstants.SHOOTER_MAX_TARGET_RPM, newTargetRpm));
    }

    /**
     * Sets whether the indexer may feed. Even when allowed, it only feeds while shooting with the wheel at speed.
     * Robot sets this every loop: while aiming, only once the robot is in a valid position and aimed; otherwise true.
     *
     * @param allowed true to let the indexer feed once the wheel is at speed; false to hold balls back
     */
    public void setFeedAllowed(boolean allowed) {
        feedAllowed = allowed;
    }

    /**
     * Tells whether the wheel is at speed (the hysteresis latch the indexer feeds on), as of the last update().
     *
     * @return true while shooting with the wheel within the feed tolerances of the target RPM
     */
    public boolean isAtSpeed() {
        return wheelAtSpeed;
    }

    /**
     * Converts a shooter wheel speed to the encoder units setVelocity() expects.
     *
     * @param rpm the shooter motor output-shaft speed, in revolutions per minute
     * @return the same speed in encoder ticks per second
     */
    private double rpmToTicksPerSecond(double rpm) {
        // * ticks-per-rev gives ticks per minute, / 60 gives ticks per second
        return rpm * RobotConstants.SHOOTER_ENCODER_TICKS_PER_REV / 60.0;
    }

    /**
     * Reads the raw encoder speed, useful for checking that the encoder is connected and counting.
     *
     * @return the shooter motor's speed in encoder ticks per second
     */
    public double getShooterTicksPerSecond() {
        return shooterMotor.getVelocity();
    }

    /**
     * The highest shooter speed seen since init, so the top speed can be read off telemetry
     * after a spin-up without having to catch it live.
     *
     * @return the peak RPM magnitude measured so far
     */
    public double getPeakShooterRpm() {
        return peakShooterRpm;
    }

    /**
     * Requests the shooting state. Robot must check that this is allowed first;
     * this method does not check the intake.
     */
    public void requestShooting() {
        state = State.SHOOTING;
    }

    /**
     * Requests the idle (stopped) state for the flywheel. The indexer stops with it.
     */
    public void requestIdle() {
        state = State.IDLE;
    }

    /**
     * Tells whether the shooter is running. Robot checks this before allowing the intake to start.
     *
     * @return true if the shooter is in the SHOOTING state
     */
    public boolean isActive() {
        return state == State.SHOOTING;
    }

    /**
     * Gets the shooter's current state, for matching against a RobotSuperState's definition
     * of what the shooter should be doing.
     *
     * @return the current state
     */
    public State getState() {
        return state;
    }
}
