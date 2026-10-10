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
    // INDEXER_REST_KD is a little damping: power per deg/s the indexer is already turning, subtracted, so it brakes as
    // it swings toward rest instead of coasting through it.
    // If it wobbles around rest, lower INDEXER_REST_KP or raise INDEXER_REST_KD; if it stops short, raise KP.
    // History: 0.004 / 0.04 with no damping reached rest but wobbled through it (8-16 crossings per 2.5 s window,
    // only 35 of 153 windows ended inside the tolerance, 10-09 11:01 logs), so P was nudged down and damping added.
    // 0.005 / 0.1 held too weakly, 0.012 / 0.2 was jerky. A 0.1 floor moves this Axon about 20 deg per loop, far
    // coarser than the 3 deg window.
    public static final double INDEXER_REST_KP = 0.0035;
    public static final double INDEXER_REST_KD = 0.0001;
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
     * far off it is, at least the minimum power so friction can't stall it short, then braked a little by how fast it
     * is already turning (the damping term), at most the maximum either way, and zero once it's within the tolerance.
     * Uses the INDEXER_REST_* constants at the top of this class.
     *
     * @param errorToRestDegrees      the turn to the nearest rest angle (see errorToNearestRestDegrees())
     * @param turnRateDegreesPerSecond how fast the indexer is turning, positive the positive-power way
     * @return the servo power, positive to turn the positive-power way
     */
    public static double powerToRest(double errorToRestDegrees, double turnRateDegreesPerSecond) {
        if (isAtRest(errorToRestDegrees)) {
            return 0.0;
        }
        double proportionalMagnitude = Math.max(INDEXER_REST_MIN_POWER,
                Math.min(INDEXER_REST_MAX_POWER, INDEXER_REST_KP * Math.abs(errorToRestDegrees)));
        double proportionalPower = Math.signum(errorToRestDegrees) * proportionalMagnitude;
        // Turning toward rest makes the damping push back against the motion, so it slows down before it gets there
        double dampedPower = proportionalPower - INDEXER_REST_KD * turnRateDegreesPerSecond;
        return Math.max(-INDEXER_REST_MAX_POWER, Math.min(INDEXER_REST_MAX_POWER, dampedPower));
    }

    /**
     * The smallest turn from one angle to another, for measuring turn rate across the encoder's 360 -> 0 wrap.
     *
     * @param fromDegrees the earlier angle
     * @param toDegrees   the later angle
     * @return the change, in [-180, 180) deg
     */
    public static double angleChangeDegrees(double fromDegrees, double toDegrees) {
        // Double modulo keeps it positive before shifting into [-180, 180)
        return ((toDegrees - fromDegrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
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
