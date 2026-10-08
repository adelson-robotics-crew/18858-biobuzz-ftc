package org.firstinspires.ftc.teamcode.tests;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerRest;
import org.junit.Test;

/**
 * Tests the indexer rest math: angles from the INIT position (including across the encoder's 360 -> 0 wrap),
 * the turn to the nearest rest angle, and the power that turns it back.
 */
public class IndexerRestTest {
    private static final double EPSILON = 1e-9;
    private static final double SPACING = 90.0;

    /**
     * The angle is measured from the INIT reading, wraps across the encoder's 360 -> 0 jump, and flips with the
     * encoder direction.
     */
    @Test
    public void angleFromStartWrapsAndFollowsDirection() {
        assertEquals(10.0, IndexerRest.angleFromStartDegrees(110.0, 100.0, 1.0), EPSILON);
        assertEquals(20.0, IndexerRest.angleFromStartDegrees(5.0, 345.0, 1.0), EPSILON);  // crossed 360 -> 0
        assertEquals(350.0, IndexerRest.angleFromStartDegrees(90.0, 100.0, 1.0), EPSILON); // turned 10 deg backward
        assertEquals(350.0, IndexerRest.angleFromStartDegrees(110.0, 100.0, -1.0), EPSILON); // encoder reversed
    }

    /**
     * The nearest rest angle is the closest multiple of the spacing, either way, including back across 0.
     */
    @Test
    public void errorPointsToTheNearestRestAngle() {
        assertEquals(0.0, IndexerRest.errorToNearestRestDegrees(0.0, SPACING), EPSILON);
        assertEquals(-10.0, IndexerRest.errorToNearestRestDegrees(10.0, SPACING), EPSILON);  // back to 0
        assertEquals(10.0, IndexerRest.errorToNearestRestDegrees(80.0, SPACING), EPSILON);   // on to 90
        assertEquals(-5.0, IndexerRest.errorToNearestRestDegrees(185.0, SPACING), EPSILON);  // back to 180
        assertEquals(5.0, IndexerRest.errorToNearestRestDegrees(355.0, SPACING), EPSILON);   // on to 360, i.e. 0
    }

    /**
     * No power inside the tolerance; outside it, at least the minimum, at most the maximum, in the error's direction.
     */
    @Test
    public void powerToRestIsBoundedAndPointsTheRightWay() {
        assertEquals(0.0, IndexerRest.powerToRest(RobotConstants.INDEXER_REST_TOLERANCE_DEG), EPSILON);
        assertEquals(0.0, IndexerRest.powerToRest(-1.0), EPSILON);
        // Just outside the tolerance: the proportional power, but never less than the minimum
        double justOutsideDegrees = RobotConstants.INDEXER_REST_TOLERANCE_DEG + 0.5;
        assertEquals(Math.max(RobotConstants.INDEXER_REST_MIN_POWER, RobotConstants.INDEXER_REST_KP * justOutsideDegrees),
                IndexerRest.powerToRest(justOutsideDegrees), EPSILON);
        assertEquals(-RobotConstants.INDEXER_REST_MAX_POWER, IndexerRest.powerToRest(-45.0), EPSILON);
        assertEquals(RobotConstants.INDEXER_REST_KP * 10.0, IndexerRest.powerToRest(10.0), EPSILON); // between min and max
    }
}
