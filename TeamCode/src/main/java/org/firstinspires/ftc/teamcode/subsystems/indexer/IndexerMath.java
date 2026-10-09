package org.firstinspires.ftc.teamcode.subsystems.indexer;

/**
 * The math for correcting the indexer to a rest angle: turning the Axon's raw encoder reading into an angle measured
 * from the saved 0 deg, how far that is from the nearest rest angle (either way), whether it's there, and the servo
 * power that turns it back. Pure math, no hardware, so it can be unit-tested. Indexer owns the servo and the encoder
 * and uses this.
 */
public final class IndexerMath {

    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Correction controller (runs for a short window before and after each shot, see Indexer): servo power per degree
    // off the nearest rest angle, in whichever direction gets there, clamped to INDEXER_REST_MAX_POWER. Within
    // INDEXER_REST_TOLERANCE_DEG it counts as at rest and the power is 0, so the servo doesn't buzz. Outside it, at
    // least INDEXER_REST_MIN_POWER is used so friction can't stall it a few degrees short.
    // If it wobbles around rest, lower INDEXER_REST_KP or INDEXER_REST_MIN_POWER; if it stops short, raise them.
    // History: 0.004 / 0.04 is the two-way setting that worked. 0.005 / 0.1 held too weakly, 0.012 / 0.2 was jerky.
    // A 0.1 floor moves this Axon about 20 deg per loop (10-09 logs), far coarser than the 3 deg window, so a
    // two-way controller with it would bounce back and forth around rest.
    public static final double INDEXER_REST_KP = 0.004;
    public static final double INDEXER_REST_MAX_POWER = 1.0;
    public static final double INDEXER_REST_MIN_POWER = 0.04;
    public static final double INDEXER_REST_TOLERANCE_DEG = 3.0;

    /**
     * Not meant to be instantiated; use the static methods.
     */
    private IndexerMath() {
    }

    /**
     * The indexer's angle measured from the saved 0 deg, in the direction positive servo power turns it.
     *
     * @param rawAngleDegrees  the encoder's angle now, from 0 to 360
     * @param zeroAngleDegrees the encoder's angle at the saved 0 deg
     * @param encoderDirection +1 if positive servo power makes the encoder angle go up, -1 if it makes it go down
     * @return the angle from 0 deg, in [0, 360)
     */
    public static double angleFromStartDegrees(double rawAngleDegrees, double zeroAngleDegrees, double encoderDirection) {
        return wrapDegrees(encoderDirection * (rawAngleDegrees - zeroAngleDegrees));
    }

    /**
     * How far the indexer is from the nearest rest angle (a multiple of the rest spacing from 0 deg), either way.
     *
     * @param angleFromStartDegrees the indexer's angle from 0 deg (see angleFromStartDegrees())
     * @param restSpacingDegrees    the gap between rest angles, e.g. 90
     * @return the turn to the nearest rest angle, in degrees: positive means the positive-power way.
     *         Never more than half the spacing either way
     */
    public static double errorToNearestRestDegrees(double angleFromStartDegrees, double restSpacingDegrees) {
        double nearestRestDegrees = Math.round(angleFromStartDegrees / restSpacingDegrees) * restSpacingDegrees;
        return nearestRestDegrees - angleFromStartDegrees;
    }

    /**
     * Whether the indexer is at a rest angle: within INDEXER_REST_TOLERANCE_DEG of one, either side.
     *
     * @param errorToRestDegrees the turn to the nearest rest angle (see errorToNearestRestDegrees())
     * @return true if it's within the tolerance
     */
    public static boolean isAtRest(double errorToRestDegrees) {
        return Math.abs(errorToRestDegrees) <= INDEXER_REST_TOLERANCE_DEG;
    }

    /**
     * The servo power that turns the indexer to the nearest rest angle, whichever way that is: proportional to how
     * far off it is, at least the minimum power so friction can't stall it short, at most the maximum, and zero
     * once it's within the tolerance. Uses the INDEXER_REST_* constants at the top of this class.
     *
     * @param errorToRestDegrees the turn to the nearest rest angle (see errorToNearestRestDegrees())
     * @return the servo power, positive to turn the positive-power way
     */
    public static double powerToRest(double errorToRestDegrees) {
        if (isAtRest(errorToRestDegrees)) {
            return 0.0;
        }
        double powerMagnitude = Math.max(INDEXER_REST_MIN_POWER,
                Math.min(INDEXER_REST_MAX_POWER, INDEXER_REST_KP * Math.abs(errorToRestDegrees)));
        return Math.signum(errorToRestDegrees) * powerMagnitude;
    }

    /**
     * Wraps an angle into [0, 360).
     *
     * @param angleDegrees any angle, in degrees
     * @return the same angle wrapped into [0, 360)
     */
    private static double wrapDegrees(double angleDegrees) {
        // Double modulo keeps the result positive for negative angles
        return (angleDegrees % 360.0 + 360.0) % 360.0;
    }
}
