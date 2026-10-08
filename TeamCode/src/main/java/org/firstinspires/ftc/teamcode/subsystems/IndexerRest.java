package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * The math for holding the indexer at a rest angle: turning the Axon's raw encoder reading into an angle measured
 * from the INIT position, finding how far that is from the nearest rest angle, and the servo power that turns it
 * back. Pure math, no hardware, so it can be unit-tested. Shooter owns the servo and the encoder and uses this.
 */
public final class IndexerRest {

    /**
     * Not meant to be instantiated; use the static methods.
     */
    private IndexerRest() {
    }

    /**
     * The indexer's angle measured from where it was at INIT, in the direction positive servo power turns it.
     *
     * @param rawAngleDegrees  the encoder's angle now, from 0 to 360
     * @param zeroAngleDegrees the encoder's angle at INIT
     * @param encoderDirection +1 if positive servo power makes the encoder angle go up, -1 if it makes it go down
     * @return the angle from the INIT position, in [0, 360)
     */
    public static double angleFromStartDegrees(double rawAngleDegrees, double zeroAngleDegrees, double encoderDirection) {
        return wrapDegrees(encoderDirection * (rawAngleDegrees - zeroAngleDegrees));
    }

    /**
     * How far the indexer is from the nearest rest angle (a multiple of the rest spacing from the INIT position),
     * as the turn that would get it there.
     *
     * @param angleFromStartDegrees the indexer's angle from the INIT position (see angleFromStartDegrees())
     * @param restSpacingDegrees    the gap between rest angles, e.g. 90
     * @return the turn to the nearest rest angle, in degrees: positive means turn the positive-power way.
     *         Never more than half the spacing either way
     */
    public static double errorToNearestRestDegrees(double angleFromStartDegrees, double restSpacingDegrees) {
        double nearestRestDegrees = Math.round(angleFromStartDegrees / restSpacingDegrees) * restSpacingDegrees;
        return nearestRestDegrees - angleFromStartDegrees;
    }

    /**
     * The servo power that turns the indexer back to rest: proportional to how far off it is, at least the minimum
     * power so friction can't stall it short, at most the maximum, and zero once it's within the tolerance.
     * Uses the INDEXER_REST_* constants in RobotConstants.
     *
     * @param errorToRestDegrees the turn to the nearest rest angle (see errorToNearestRestDegrees())
     * @return the servo power, positive to turn the positive-power way
     */
    public static double powerToRest(double errorToRestDegrees) {
        double errorMagnitudeDegrees = Math.abs(errorToRestDegrees);
        if (errorMagnitudeDegrees <= RobotConstants.INDEXER_REST_TOLERANCE_DEG) {
            return 0.0;
        }
        double powerMagnitude = Math.max(RobotConstants.INDEXER_REST_MIN_POWER,
                Math.min(RobotConstants.INDEXER_REST_MAX_POWER, RobotConstants.INDEXER_REST_KP * errorMagnitudeDegrees));
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
