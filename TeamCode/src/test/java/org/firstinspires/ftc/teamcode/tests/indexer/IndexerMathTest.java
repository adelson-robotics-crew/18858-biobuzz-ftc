package org.firstinspires.ftc.teamcode.tests.indexer;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerMath;
import org.junit.Test;

/**
 * Tests the indexer rest math: angles from the saved 0 deg (including across the encoder's 360 -> 0 wrap), the
 * turn to the nearest rest angle, the at-rest check, and the two-way correction power.
 */
public class IndexerMathTest {
    private static final double EPSILON = 1e-9;
    private static final double SPACING = 90.0;

    /**
     * The angle is measured from the saved 0 deg reading, wraps across the encoder's 360 -> 0 jump, and flips with
     * the encoder direction.
     */
    @Test
    public void angleFromStartWrapsAndFollowsDirection() {
        assertEquals(10.0, IndexerMath.angleFromStartDegrees(110.0, 100.0, 1.0), EPSILON);
        assertEquals(20.0, IndexerMath.angleFromStartDegrees(5.0, 345.0, 1.0), EPSILON);  // crossed 360 -> 0
        assertEquals(350.0, IndexerMath.angleFromStartDegrees(90.0, 100.0, 1.0), EPSILON); // turned 10 deg backward
        assertEquals(350.0, IndexerMath.angleFromStartDegrees(110.0, 100.0, -1.0), EPSILON); // encoder reversed
    }

    /**
     * The nearest rest angle is the closest multiple of the spacing, either way, including back across 0.
     */
    @Test
    public void errorPointsToTheNearestRestAngle() {
        assertEquals(0.0, IndexerMath.errorToNearestRestDegrees(0.0, SPACING), EPSILON);
        assertEquals(-10.0, IndexerMath.errorToNearestRestDegrees(10.0, SPACING), EPSILON);  // back to 0
        assertEquals(10.0, IndexerMath.errorToNearestRestDegrees(80.0, SPACING), EPSILON);   // on to 90
        assertEquals(-5.0, IndexerMath.errorToNearestRestDegrees(185.0, SPACING), EPSILON);  // back to 180
        assertEquals(5.0, IndexerMath.errorToNearestRestDegrees(355.0, SPACING), EPSILON);   // on to 360, i.e. 0
    }

    /**
     * At rest within the tolerance on either side of a rest angle.
     */
    @Test
    public void atRestWithinTheToleranceEitherSide() {
        double tolerance = IndexerMath.INDEXER_REST_TOLERANCE_DEG;
        assertTrue(IndexerMath.isAtRest(0.0));
        assertTrue(IndexerMath.isAtRest(tolerance));
        assertTrue(IndexerMath.isAtRest(-tolerance));
        assertFalse(IndexerMath.isAtRest(tolerance + 0.5));
        assertFalse(IndexerMath.isAtRest(-tolerance - 0.5));
    }

    /**
     * No power inside the tolerance; outside it, at least the minimum, at most the maximum, and in the error's
     * direction, so it corrects either way.
     */
    @Test
    public void powerToRestIsBoundedAndCorrectsEitherWay() {
        assertEquals(0.0, IndexerMath.powerToRest(IndexerMath.INDEXER_REST_TOLERANCE_DEG), EPSILON);
        // Just outside the tolerance: the proportional power, but never less than the minimum
        double justOutsideDegrees = IndexerMath.INDEXER_REST_TOLERANCE_DEG + 0.5;
        double expectedNearMagnitude = Math.max(IndexerMath.INDEXER_REST_MIN_POWER,
                IndexerMath.INDEXER_REST_KP * justOutsideDegrees);
        assertEquals(expectedNearMagnitude, IndexerMath.powerToRest(justOutsideDegrees), EPSILON);
        assertEquals(-expectedNearMagnitude, IndexerMath.powerToRest(-justOutsideDegrees), EPSILON);
        // The farthest it can be from rest (half the spacing): proportional, capped at the maximum
        double farMagnitude = Math.min(IndexerMath.INDEXER_REST_MAX_POWER, IndexerMath.INDEXER_REST_KP * 45.0);
        assertEquals(-farMagnitude, IndexerMath.powerToRest(-45.0), EPSILON);
        assertEquals(farMagnitude, IndexerMath.powerToRest(45.0), EPSILON);
    }
}
