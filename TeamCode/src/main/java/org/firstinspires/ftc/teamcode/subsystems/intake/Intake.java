package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * The intake: two continuous-rotation servos (one on each side) pulling balls in, and a DC motor.
 * Only Robot decides when this may run (see Robot.requestIntake()); this class doesn't know Shooter exists.
 */
public class Intake {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Hardware names: must match the Robot Controller's hardware configuration.
    private static final String RIGHT_INTAKE_SERVO_NAME = "intake_right"; // continuous-rotation servo
    private static final String LEFT_INTAKE_SERVO_NAME = "intake_left";   // continuous-rotation servo
    private static final String INTAKE_MOTOR_NAME = "intake";             // DC motor

    // The two intake servos face opposite ways, so pulling a ball in needs opposite signs.
    // If a servo pushes the ball out instead of in, flip that servo's sign.
    private static final double INTAKE_RIGHT_SERVO_POWER = -1.0;
    private static final double INTAKE_LEFT_SERVO_POWER = 1.0;
    private static final double INTAKE_MOTOR_POWER = 1.0;

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
        rightServo = hardwareMap.get(CRServo.class, RIGHT_INTAKE_SERVO_NAME);
        leftServo = hardwareMap.get(CRServo.class, LEFT_INTAKE_SERVO_NAME);
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
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
                rightServo.setPower(INTAKE_RIGHT_SERVO_POWER);
                leftServo.setPower(INTAKE_LEFT_SERVO_POWER);
                intakeMotor.setPower(INTAKE_MOTOR_POWER);
                break;
            case OUTTAKING:
                // The intake powers negated: everything spins the opposite way, pushing balls back out
                rightServo.setPower(-INTAKE_RIGHT_SERVO_POWER);
                leftServo.setPower(-INTAKE_LEFT_SERVO_POWER);
                intakeMotor.setPower(-INTAKE_MOTOR_POWER);
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
