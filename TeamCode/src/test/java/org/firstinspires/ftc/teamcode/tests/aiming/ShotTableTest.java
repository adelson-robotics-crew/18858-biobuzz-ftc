package org.firstinspires.ftc.teamcode.tests.aiming;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTarget;
import org.junit.Test;

/**
 * Tests the distance -> RPM table against its own rows and against the raw shot measurements.
 */
public class ShotTableTest {
    private static final double RPM_EPSILON = 1e-9;
    private final ShotTable shotTable = ShotTable.fromTunedRows();

    /**
     * Each row's distance gives exactly that row's RPM.
     */
    @Test
    public void exactValueAtEachRow() {
        for (int rowIndex = 0; rowIndex < ShotTable.SHOT_TABLE_DISTANCES_INCHES.length; rowIndex++) {
            assertEquals(ShotTable.SHOT_TABLE_RPMS[rowIndex],
                    shotTable.rpmAt(ShotTable.SHOT_TABLE_DISTANCES_INCHES[rowIndex]), RPM_EPSILON);
        }
    }

    /**
     * Halfway between two rows gives the average of their RPMs.
     */
    @Test
    public void midpointsInterpolateLinearly() {
        assertEquals(2412.5, shotTable.rpmAt(43.5), RPM_EPSILON);  // 2450 -> 2375
        assertEquals(2395.0, shotTable.rpmAt(46.5), RPM_EPSILON);  // 2375 -> 2415
        assertEquals(2425.0, shotTable.rpmAt(50.0), RPM_EPSILON);  // 2415 -> 2435
        assertEquals(2467.5, shotTable.rpmAt(57.2), RPM_EPSILON);  // 2435 -> 2500
    }

    /**
     * Past either end, the table holds the end row's RPM instead of extrapolating.
     */
    @Test
    public void doesNotExtrapolatePastEnds() {
        assertEquals(2450.0, shotTable.rpmAt(30.0), RPM_EPSILON);
        assertEquals(2500.0, shotTable.rpmAt(80.0), RPM_EPSILON);
    }

    /**
     * The range edges are inclusive, and just outside them is out of range.
     */
    @Test
    public void rangeEdges() {
        assertFalse(shotTable.inRange(42.5));
        assertTrue(shotTable.inRange(43.0));
        assertTrue(shotTable.inRange(62.4));
        assertFalse(shotTable.inRange(63.0));
    }

    /**
     * At every measured shot's distance, the table's RPM is inside the RPM window that worked there.
     */
    @Test
    public void tableRpmFallsInsideEveryMeasuredWindow() {
        for (double[] row : ShotMeasurements.ROWS) {
            double distanceInches = Math.hypot(ShotTarget.BLUE_LOWER.xInches - ShotMeasurements.pedroX(row),
                    ShotTarget.BLUE_LOWER.yInches - ShotMeasurements.pedroY(row));
            double tableRpm = shotTable.rpmAt(distanceInches);
            assertTrue(ShotMeasurements.describe(row) + " table RPM " + tableRpm + " outside window",
                    tableRpm >= row[ShotMeasurements.MIN_RPM] && tableRpm <= row[ShotMeasurements.MAX_RPM]);
        }
    }
}
