package org.firstinspires.ftc.teamcode.aiming;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Tests the shot geometry (heading and angle off center) against the raw shot measurements.
 */
public class ShotSolverTest {
    private static final double MEASURED_HEADING_TOLERANCE_DEG = 5.0;
    private static final double ANGLE_EPSILON_DEG = 0.1;

    /**
     * The computed shooting heading is within 5 deg of the heading every measured shot actually used.
     */
    @Test
    public void headingMatchesEveryMeasuredShot() {
        for (double[] row : ShotMeasurements.ROWS) {
            ShotSolution solution = ShotSolver.solve(row[ShotMeasurements.X], row[ShotMeasurements.Y]);
            double headingDifferenceDeg = wrapDegrees(
                    Math.toDegrees(solution.targetHeadingRadians) - row[ShotMeasurements.HEADING_DEG]);
            assertTrue(ShotMeasurements.describe(row) + " heading off by " + headingDifferenceDeg,
                    Math.abs(headingDifferenceDeg) <= MEASURED_HEADING_TOLERANCE_DEG);
        }
    }

    /**
     * The target heading is wrapped into [-PI, PI).
     */
    @Test
    public void targetHeadingIsWrapped() {
        for (double[] row : ShotMeasurements.ROWS) {
            double targetHeadingRadians = ShotSolver.solve(row[ShotMeasurements.X], row[ShotMeasurements.Y]).targetHeadingRadians;
            assertTrue(targetHeadingRadians >= -Math.PI && targetHeadingRadians < Math.PI);
        }
    }

    /**
     * Every measured point is inside the angle limit except (26.15, 6.34), which is outside on purpose.
     */
    @Test
    public void measuredPointsAngleLimit() {
        for (double[] row : ShotMeasurements.ROWS) {
            ShotSolution solution = ShotSolver.solve(row[ShotMeasurements.X], row[ShotMeasurements.Y]);
            boolean isTheWidePoint = row[ShotMeasurements.X] == 26.15 && row[ShotMeasurements.Y] == 6.34;
            assertEquals(ShotMeasurements.describe(row) + " angle " + solution.angleOffCenterDegrees,
                    !isTheWidePoint, Math.abs(solution.angleOffCenterDegrees) <= 45.0);
        }
        ShotSolution widePoint = ShotSolver.solve(26.15, 6.34);
        assertEquals(-48.6, widePoint.angleOffCenterDegrees, ANGLE_EPSILON_DEG);
        assertEquals(ShotSolution.Validity.ANGLE_TOO_WIDE, widePoint.validity);
    }

    /**
     * Spot checks: nearly straight in front, and the wide point mirrored to the right side.
     */
    @Test
    public void angleOffCenterSpotChecks() {
        assertEquals(0.7, ShotSolver.solve(73.5, 2.88).angleOffCenterDegrees, ANGLE_EPSILON_DEG);
        ShotSolution mirroredWidePoint = ShotSolver.solve(119.75, 6.34);
        assertEquals(48.6, mirroredWidePoint.angleOffCenterDegrees, ANGLE_EPSILON_DEG);
        assertEquals(ShotSolution.Validity.ANGLE_TOO_WIDE, mirroredWidePoint.validity);
    }

    /**
     * Invalid positions report the right reason, and a good position is valid.
     */
    @Test
    public void validityReasons() {
        ShotSolution valid = ShotSolver.solve(73.5, 2.88); // ~44.7 in, straight in front
        assertEquals(ShotSolution.Validity.VALID, valid.validity);
        assertTrue(valid.inRange);
        assertEquals(ShotSolution.Validity.TOO_CLOSE, ShotSolver.solve(73.5, 5.6).validity); // ~42 in
        assertEquals(ShotSolution.Validity.TOO_FAR, ShotSolver.solve(73.5, -20.0).validity);  // ~67.6 in
    }

    /**
     * Wraps an angle into [-180, 180).
     *
     * @param angleDegrees any angle, in degrees
     * @return the same angle wrapped into [-180, 180)
     */
    private static double wrapDegrees(double angleDegrees) {
        // Double modulo keeps the result positive before shifting back down by 180
        return ((angleDegrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
