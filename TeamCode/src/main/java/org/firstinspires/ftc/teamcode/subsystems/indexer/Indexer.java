package org.firstinspires.ftc.teamcode.subsystems.indexer;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * The indexer: an Axon servo in continuous mode that feeds balls into the shooter wheel, plus its feedback wire
 * (an analog input) that reports its angle within one turn. It only ever spins continuously the feeding way;
 * the correction PID may nudge it either way. States:
 *   HOLDING:    no power, so it doesn't get corrected every time the robot accelerates or turns. The normal state.
 *   FEEDING:    spins at INDEXER_SERVO_POWER, pushing balls into the wheel.
 *   CORRECTING: the PID turns it to the nearest rest angle, either way, for a set time, then -> HOLDING. Rest angles
 *               are every INDEXER_REST_SPACING_DEG from the saved 0 deg. It corrects twice per shot:
 *               - before: for INDEXER_CORRECT_BEFORE_SHOT_SECONDS as the shooter starts spinning up
 *                 (prepareForShot()), since it gets knocked off rest while unpowered, and it won't feed until the
 *                 wheel is at speed anyway.
 *               - after: for INDEXER_CORRECT_AFTER_SHOT_SECONDS once feeding stops (stopFeeding()).
 * Only Robot decides when it feeds and corrects (see Robot.update()); this class doesn't know the shooter exists.
 * The math is in IndexerMath so it can be unit-tested without hardware.
 */
public class Indexer {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Hardware names: must match the Robot Controller's hardware configuration.
    private static final String INDEXER_SERVO_NAME = "indexer";           // Axon servo in continuous mode (configure as a Continuous Rotation Servo)
    private static final String INDEXER_ENCODER_NAME = "indexer_encoder"; // the Axon's position feedback wire, on analog port 0 (configure as an Analog Input)

    // Power the indexer spins at while feeding (the sign is the feeding direction). The Axon is brushless and fast,
    // so this is kept low
    private static final double INDEXER_SERVO_POWER = -0.2;

    // Indexer rest positions, which the correction PID turns it to. Angles are measured from where the
    // indexer was at the FIRST INIT after the Robot Controller app started (that's 0 deg, so put it in a rest
    // position before that first INIT); every OpMode after that reuses the same 0 deg (see savedZeroAngleDegrees).
    // There's a rest angle every INDEXER_REST_SPACING_DEG (0, 90, 180, 270).
    private static final double INDEXER_REST_SPACING_DEG = 90.0;
    // +1.0 if positive servo power makes the encoder angle go up, -1.0 if it makes it go down. If the indexer won't
    // settle at rest (it keeps spinning or runs away when it should stop), flip this.
    // -1.0: the 12:56 DriveTest log showed the angle going up while feeding at negative power. With +1.0 the rest
    // controller pushed away from rest and swung back and forth by up to 45 deg.
    private static final double INDEXER_ENCODER_DIRECTION = -1.0;
    // The Axon's feedback voltage covers one full turn (0 V to the hub's max analog voltage = 0 to 360 deg).
    private static final double INDEXER_ENCODER_DEGREES_PER_TURN = 360.0;
    // How long the correction PID runs: after feeding stops, and right before a shot (as the shooter starts spinning
    // up; the wheel takes longer than this to reach speed, so it's done before feeding starts). Then power is cut.
    private static final double INDEXER_CORRECT_AFTER_SHOT_SECONDS = 2.5;
    private static final double INDEXER_CORRECT_BEFORE_SHOT_SECONDS = 1.0;

    public enum State {
        HOLDING,
        FEEDING,
        CORRECTING
    }

    // The encoder's angle at the first INIT since the app started, which every OpMode measures rest angles from.
    // Static so it survives from one OpMode to the next (auto into TeleOp, and every re-INIT), so the indexer always
    // returns to the same home even if an OpMode ended with it off rest. A power cycle or app restart clears it.
    // Null until the first INIT records it
    private static Double savedZeroAngleDegrees = null;

    private final CRServo indexerServo;
    private final AnalogInput indexerEncoder; // the Axon's position feedback: 0 V to max voltage = one full turn
    private final double zeroAngleDegrees;    // this OpMode's copy of savedZeroAngleDegrees

    // Placed at a rest angle before INIT, so it starts holding there
    private State state = State.HOLDING;
    // When the current correction window started, and how long it lasts; see startCorrecting()
    private final ElapsedTime correctionTimer = new ElapsedTime();
    private double correctionSeconds = 0.0;
    // The angle and time of the previous correcting loop, for the damping term's turn rate; NaN on the first loop
    // of a window, when there's no previous reading yet
    private double previousAngleDegrees = Double.NaN;
    private final ElapsedTime timeSincePreviousAngle = new ElapsedTime();

    /**
     * Gets the indexer hardware from the hardware map. On the first INIT since the app started, records the current
     * angle as 0 deg (so the indexer should be in a rest position then); every later INIT reuses that same 0 deg.
     *
     * @param hardwareMap the hardware map from the running OpMode
     */
    public Indexer(HardwareMap hardwareMap) {
        indexerServo = hardwareMap.get(CRServo.class, INDEXER_SERVO_NAME);
        indexerEncoder = hardwareMap.get(AnalogInput.class, INDEXER_ENCODER_NAME);
        if (savedZeroAngleDegrees == null) {
            savedZeroAngleDegrees = readEncoderDegrees();
        }
        zeroAngleDegrees = savedZeroAngleDegrees;
    }

    /**
     * Applies power for the current state. CORRECTING runs the PID until its window runs out, then cuts power and
     * moves to HOLDING. Call once per loop.
     */
    public void update() {
        switch (state) {
            case HOLDING:
                indexerServo.setPower(0.0); // the unpowered servo holds its position
                break;
            case FEEDING:
                indexerServo.setPower(INDEXER_SERVO_POWER);
                break;
            case CORRECTING:
                if (correctionTimer.seconds() > correctionSeconds) {
                    state = State.HOLDING;
                    indexerServo.setPower(0.0);
                } else {
                    double angleDegrees = getAngleDegrees();
                    double turnRateDegreesPerSecond = 0.0;
                    double secondsSincePrevious = timeSincePreviousAngle.seconds();
                    if (!Double.isNaN(previousAngleDegrees) && secondsSincePrevious > 0.0) {
                        turnRateDegreesPerSecond = IndexerMath.angleChangeDegrees(previousAngleDegrees, angleDegrees)
                                / secondsSincePrevious;
                    }
                    previousAngleDegrees = angleDegrees;
                    timeSincePreviousAngle.reset();
                    double errorDegrees = IndexerMath.errorToNearestRestDegrees(angleDegrees, INDEXER_REST_SPACING_DEG);
                    indexerServo.setPower(IndexerMath.powerToRest(errorDegrees, turnRateDegreesPerSecond));
                }
                break;
        }
    }

    /**
     * Requests the feeding state. Robot must check that the shooter is ready first; this method does not.
     */
    public void requestFeeding() {
        state = State.FEEDING;
    }

    /**
     * Stops feeding: if it was feeding, the PID corrects it to the nearest rest angle for
     * INDEXER_CORRECT_AFTER_SHOT_SECONDS, then cuts power. If it wasn't feeding, nothing changes, so a correction
     * already under way finishes. Safe to call every loop.
     */
    public void stopFeeding() {
        if (state == State.FEEDING) {
            startCorrecting(INDEXER_CORRECT_AFTER_SHOT_SECONDS);
        }
    }

    /**
     * Gets ready for a shot: the PID corrects it to the nearest rest angle for INDEXER_CORRECT_BEFORE_SHOT_SECONDS
     * while the shooter spins up. Robot calls this once, when the shooter starts. Does nothing while feeding.
     */
    public void prepareForShot() {
        if (state != State.FEEDING) {
            startCorrecting(INDEXER_CORRECT_BEFORE_SHOT_SECONDS);
        }
    }

    /**
     * Starts a correction window, unless one is already running that ends later (a shorter request never cuts a
     * longer window short).
     *
     * @param seconds how long the PID should run from now
     */
    private void startCorrecting(double seconds) {
        double secondsLeftInCurrentWindow = correctionSeconds - correctionTimer.seconds();
        boolean alreadyCorrectingLonger = state == State.CORRECTING && secondsLeftInCurrentWindow >= seconds;
        if (!alreadyCorrectingLonger) {
            state = State.CORRECTING;
            correctionTimer.reset();
            correctionSeconds = seconds;
            previousAngleDegrees = Double.NaN; // no turn rate until the window's second loop
        }
    }

    /**
     * Tells whether the indexer is settled at a rest angle (within the tolerance) and not feeding. An autonomous
     * waits for this after shooting, so it never moves on, or ends, with the indexer off rest. The PID may still be
     * running out its window when this turns true; that's fine, it just keeps it there.
     *
     * @return true if it's not feeding and is within the tolerance of a rest angle
     */
    public boolean isAtRest() {
        return state != State.FEEDING && IndexerMath.isAtRest(getErrorToRestDegrees());
    }

    /**
     * Tells whether the indexer is feeding balls.
     *
     * @return true in the FEEDING state; false while it's correcting or holding
     */
    public boolean isFeeding() {
        return state == State.FEEDING;
    }

    /**
     * Gets the indexer's current state.
     *
     * @return the current state
     */
    public State getState() {
        return state;
    }

    /**
     * The indexer's angle from where it was at INIT, in the direction positive servo power turns it.
     *
     * @return the angle, in [0, 360) deg
     */
    public double getAngleDegrees() {
        return IndexerMath.angleFromStartDegrees(readEncoderDegrees(), zeroAngleDegrees,
                INDEXER_ENCODER_DIRECTION);
    }

    /**
     * How far the indexer is from the nearest rest angle.
     *
     * @return the turn to the nearest rest angle, in degrees (positive = the positive-power way)
     */
    public double getErrorToRestDegrees() {
        return IndexerMath.errorToNearestRestDegrees(getAngleDegrees(), INDEXER_REST_SPACING_DEG);
    }

    /**
     * The raw voltage on the indexer's feedback wire, for checking the wiring: it should sweep from about 0 V up to
     * the hub's max analog voltage (about 3.3 V) as the indexer turns once. Stuck near 0 V means no signal.
     *
     * @return the feedback voltage, in volts
     */
    public double getEncoderVoltage() {
        return indexerEncoder.getVoltage();
    }

    /**
     * Reads the indexer's raw angle from the Axon's feedback voltage.
     *
     * @return the encoder angle, from 0 to 360 deg
     */
    private double readEncoderDegrees() {
        // The feedback voltage sweeps from 0 to the hub's max analog voltage over one turn
        return indexerEncoder.getVoltage() / indexerEncoder.getMaxVoltage()
                * INDEXER_ENCODER_DEGREES_PER_TURN;
    }
}
