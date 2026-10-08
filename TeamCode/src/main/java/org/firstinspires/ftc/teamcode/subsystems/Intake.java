package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * The intake: two continuous-rotation servos (one on each side) pulling balls in, and a DC motor.
 * Only Robot decides when this may run (see Robot.requestIntake()); this class doesn't know Shooter exists.
 */
public class Intake {
    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    private final CRServo rightServo;
    private final CRServo leftServo;
    private final DcMotor intakeMotor;

    private State state = State.IDLE;

    /**
     * Gets the intake hardware from the hardware map.
     *
     * @param hardwareMap the hardware map from the running OpMode
     */
    public Intake(HardwareMap hardwareMap) {
        rightServo = hardwareMap.get(CRServo.class, RobotConstants.RIGHT_INTAKE_SERVO_NAME);
        leftServo = hardwareMap.get(CRServo.class, RobotConstants.LEFT_INTAKE_SERVO_NAME);
        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_MOTOR_NAME);
    }

    /**
     * Applies power for the current state. Call once per loop.
     */
    public void update() {
        switch (state) {
            case IDLE:
                rightServo.setPower(0.0);
                leftServo.setPower(0.0);
                intakeMotor.setPower(0.0);
                break;
            case INTAKING:
                rightServo.setPower(RobotConstants.INTAKE_RIGHT_SERVO_POWER);
                leftServo.setPower(RobotConstants.INTAKE_LEFT_SERVO_POWER);
                intakeMotor.setPower(RobotConstants.INTAKE_MOTOR_POWER);
                break;
            case OUTTAKING:
                // The intake powers negated: everything spins the opposite way, pushing balls back out
                rightServo.setPower(-RobotConstants.INTAKE_RIGHT_SERVO_POWER);
                leftServo.setPower(-RobotConstants.INTAKE_LEFT_SERVO_POWER);
                intakeMotor.setPower(-RobotConstants.INTAKE_MOTOR_POWER);
                break;
        }
    }

    /**
     * Requests the intaking state. Robot must check that this is allowed first;
     * this method does not check the shooter.
     */
    public void requestIntaking() {
        state = State.INTAKING;
    }

    /**
     * Requests the outtaking state (the intake run backwards). Robot must check that this is allowed first;
     * this method does not check the shooter.
     */
    public void requestOuttaking() {
        state = State.OUTTAKING;
    }

    /**
     * Requests the idle (stopped) state.
     */
    public void requestIdle() {
        state = State.IDLE;
    }

    /**
     * Tells whether the intake is running. Robot checks this before allowing the shooter to start.
     *
     * @return true if the intake is running in either direction (INTAKING or OUTTAKING)
     */
    public boolean isActive() {
        return state != State.IDLE;
    }

    /**
     * Gets the intake's current state, for matching against a RobotSuperState's definition
     * of what the intake should be doing.
     *
     * @return the current state
     */
    public State getState() {
        return state;
    }
}
