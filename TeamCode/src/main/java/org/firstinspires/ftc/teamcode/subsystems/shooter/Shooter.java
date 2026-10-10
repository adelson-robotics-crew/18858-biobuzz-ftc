package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * The shooter: a DC motor that spins the shooting wheel. Only Robot decides when it may run (see
 * Robot.requestShoot()), and when the indexer feeds balls into it (see Robot.update(), which reads isAtSpeed()).
 * While shooting, the wheel is held at the target RPM with closed-loop velocity control, so its speed
 * doesn't drop as the battery drains.
 * "At speed" has hysteresis: the wheel must get within INDEXER_START_FEED_RPM_TOLERANCE of the
 * target to count, and stops counting once it's more than INDEXER_STOP_FEED_RPM_TOLERANCE away.
 * So a ball is never fed into a wheel that is too slow (still spinning up, coasting down) or too fast
 * (overshooting after a spin-up or a target change), and the indexer doesn't flicker when a ball briefly drags
 * the wheel down.
 * The target RPM starts at RobotConstants.SHOOTER_TARGET_RPM and can be adjusted at runtime with adjustTargetRpm()
 * (so the right shooting speed can be found on the field) or set outright with setTargetRpm().
 */
public class Shooter {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Hardware names: must match the Robot Controller's hardware configuration.
    private static final String SHOOTER_MOTOR_NAME = "shooter";           // DC motor

    // Indexer feed hysteresis. While shooting, the wheel counts as "at speed" once it gets within
    // INDEXER_START_FEED_RPM_TOLERANCE of the target RPM (above or below), and keeps counting as at speed until it
    // drifts more than INDEXER_STOP_FEED_RPM_TOLERANCE away. The gap keeps the indexer from flickering on and off
    // when the wheel sags a little as a ball goes through. The indexer feeds only while the wheel is at speed.
    // The hub reports velocity in steps of 20 ticks/s, which with 28 ticks/rev is 42.9 RPM per step, so the speed
    // only ever reads in 43 RPM jumps (2357, 2400, 2443...). The tolerances are one and two of those steps: a single
    // measurement step no longer pauses feeding (25 / 50 did, 90 times in the 10-09 TeleOp log), while a real ball
    // dip (100-190 RPM in that log) still does.
    private static final double INDEXER_START_FEED_RPM_TOLERANCE = 45.0;
    private static final double INDEXER_STOP_FEED_RPM_TOLERANCE = 90.0;

    // Velocity PIDF gains the motor controller uses to hold the shooter wheel at the target RPM (setVelocity()).
    // While SHOOTER_USE_CUSTOM_VELOCITY_PIDF is false, the controller keeps its default gains and these four
    // numbers are ignored. The values below are the REV hub firmware's defaults (the goBILDA motor type in the
    // SDK doesn't set its own), so turning the flag on with them unchanged behaves the same as leaving it off.
    // Units are the hub's own (error in encoder ticks per second), not RPM.
    // Overshoot/oscillation: lower P, raise D, and also check I (it keeps pushing until the error is gone, which
    // adds overshoot on a heavy wheel). F is a feedforward: power applied in proportion to the target speed.
    private static final boolean SHOOTER_USE_CUSTOM_VELOCITY_PIDF = true;

    private static final double SHOOTER_VELOCITY_F = 13.0;
    private static final double SHOOTER_VELOCITY_P = 300.0;
    private static final double SHOOTER_VELOCITY_I = 0.00;
    private static final double SHOOTER_VELOCITY_D = 0.0;

    // Encoder ticks per one revolution of the shooter motor's output shaft.
    // The shooter is a goBILDA 5203 Yellow Jacket 6000 RPM (5203-2402-0001, 1:1, no gearbox), so the encoder's
    // 28 ticks per turn of the motor shaft are also 28 ticks per turn of the output shaft.
    private static final double SHOOTER_ENCODER_TICKS_PER_REV = 28.0;

    public enum State {
        IDLE,
        SHOOTING
    }

    private final DcMotorEx shooterMotor; // DcMotorEx (not DcMotor) so the encoder velocity can be read and set
    private State state = State.IDLE;
    private double peakShooterRpm = 0.0; // highest RPM magnitude seen since init, for finding the motor's max speed
    private double targetRpm = RobotConstants.SHOOTER_TARGET_RPM; // wheel speed held while shooting
    private boolean wheelAtSpeed = false; // the hysteresis latch: true once within the start tolerance, until past the stop tolerance

    /**
     * Gets the shooter hardware from the hardware map.
     *
     * @param hardwareMap the hardware map from the running OpMode
     */
    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        applyVelocityControlSettings();
    }

    /**
     * Sends the hub the run mode and velocity PIDF gains setVelocity() needs. Done at INIT and again at the start of
     * every shot (see requestShooting()): the shooter is on Expansion Hub 2, and when that hub loses power mid-match
     * it reboots with its default gains, which overshoot badly (10-09 logs). Re-sending them each shot means a power
     * loss only spoils the shot in progress, not the rest of the match.
     */
    private void applyVelocityControlSettings() {
        // Lets the motor controller use the encoder to hold the speed passed to setVelocity()
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (SHOOTER_USE_CUSTOM_VELOCITY_PIDF) {
            // Replaces the controller's default velocity gains for RUN_USING_ENCODER (the mode setVelocity() uses)
            shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    SHOOTER_VELOCITY_P,
                    SHOOTER_VELOCITY_I,
                    SHOOTER_VELOCITY_D,
                    SHOOTER_VELOCITY_F));
        }
    }

    /**
     * Applies power (or target velocity) for the current state, then updates whether the wheel is at speed.
     * Call once per loop.
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
            wheelAtSpeed = rpmError <= INDEXER_STOP_FEED_RPM_TOLERANCE;
        } else {
            wheelAtSpeed = rpmError <= INDEXER_START_FEED_RPM_TOLERANCE;
        }
    }

    /**
     * Reads the shooter wheel's current speed from the motor encoder.
     *
     * @return the shooter motor's speed in revolutions per minute; negative when spinning backward
     */
    public double getShooterRpm() {
        // getVelocity() is in encoder ticks per second; * 60 gives ticks per minute, / ticks-per-rev gives RPM
        return shooterMotor.getVelocity() * 60.0 / SHOOTER_ENCODER_TICKS_PER_REV;
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
        setTargetRpm(targetRpm + rpmChange);
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
        return rpm * SHOOTER_ENCODER_TICKS_PER_REV / 60.0;
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
     * Reads back the velocity PIDF gains the motor controller (the hub) is actually using for setVelocity(), to
     * check whether the SHOOTER_VELOCITY_* gains set in the constructor really took. A hub read, so call it rarely
     * (e.g. once at INIT), not every loop.
     *
     * @return the hub's RUN_USING_ENCODER PIDF coefficients
     */
    public PIDFCoefficients readHubVelocityPidf() {
        return shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * The velocity PIDF gains this class asks the hub to use (whether or not the hub took them).
     *
     * @return the configured gains, or null if SHOOTER_USE_CUSTOM_VELOCITY_PIDF is off and the hub keeps its defaults
     */
    public PIDFCoefficients getConfiguredVelocityPidf() {
        if (!SHOOTER_USE_CUSTOM_VELOCITY_PIDF) {
            return null;
        }
        return new PIDFCoefficients(SHOOTER_VELOCITY_P, SHOOTER_VELOCITY_I, SHOOTER_VELOCITY_D, SHOOTER_VELOCITY_F);
    }

    /**
     * The current the shooter motor is drawing, which shows how hard the controller is pushing (e.g. while
     * recovering from a ball). A hub read, about a couple of milliseconds.
     *
     * @return the motor current, in amps
     */
    public double getMotorCurrentAmps() {
        return shooterMotor.getCurrent(CurrentUnit.AMPS);
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
     * this method does not check the intake. When a shot starts (idle -> shooting), the velocity gains are sent to
     * the hub again, in case it lost power and forgot them (see applyVelocityControlSettings()). Safe to call every
     * loop: they're only re-sent once per shot.
     */
    public void requestShooting() {
        if (state != State.SHOOTING) {
            applyVelocityControlSettings();
        }
        state = State.SHOOTING;
    }

    /**
     * Requests the idle (stopped) state.
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
