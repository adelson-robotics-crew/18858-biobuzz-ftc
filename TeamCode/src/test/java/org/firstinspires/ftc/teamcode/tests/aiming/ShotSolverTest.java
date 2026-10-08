package org.firstinspires.ftc.teamcode.tests.aiming;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTarget;
import org.junit.Test;

/**
 * Tests the shot geometry (heading and angle off center) against the raw shot measurements, which were all taken
 * against the measured cell (ShotTarget.BLUE_LOWER). Positions are Pedro field coordinates; the spot checks are
 * written as old-frame numbers plus OFFSET so they read like the measurements.
 */
public class ShotSolverTest {
    private static final double MEASURED_HEADING_TOLERANCE_DEG = 5.0;
    private static final double ANGLE_EPSILON_DEG = 0.1;
    private static final double OFFSET = ShotMeasurements.OLD_FRAME_TO_PEDRO_OFFSET_INCHES;
    private static final ShotTarget MEASURED_CELL = ShotTarget.BLUE_LOWER;

    /**
     * The computed shooting heading is within 5 deg of the heading every measured shot actually used.
     */
    @Test
    public void headingMatchesEveryMeasuredShot() {
        for (double[] row : ShotMeasurements.ROWS) {
            ShotSolution solution = ShotSolver.solve(ShotMeasurements.pedroX(row), ShotMeasurements.pedroY(row), MEASURED_CELL);
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
            double targetHeadingRadians = ShotSolver.solve(ShotMeasurements.pedroX(row), ShotMeasurements.pedroY(row), MEASURED_CELL).targetHeadingRadians;
            assertTrue(targetHeadingRadians >= -Math.PI && targetHeadingRadians < Math.PI);
        }
    }

    /**
     * Every measured point is inside the angle limit except (26.15, 6.34), which is outside on purpose.
     */
    @Test
    public void measuredPointsAngleLimit() {
        for (double[] row : ShotMeasurements.ROWS) {
            ShotSolution solution = ShotSolver.solve(ShotMeasurements.pedroX(row), ShotMeasurements.pedroY(row), MEASURED_CELL);
            boolean isTheWidePoint = row[ShotMeasurements.X] == 26.15 && row[ShotMeasurements.Y] == 6.34;
            assertEquals(ShotMeasurements.describe(row) + " angle " + solution.angleOffCenterDegrees,
                    !isTheWidePoint, Math.abs(solution.angleOffCenterDegrees) <= 45.0);
        }
        ShotSolution widePoint = ShotSolver.solve(26.15 + OFFSET, 6.34 + OFFSET, MEASURED_CELL);
        assertEquals(-48.6, widePoint.angleOffCenterDegrees, ANGLE_EPSILON_DEG);
        assertEquals(ShotSolution.Validity.ANGLE_TOO_WIDE, widePoint.validity);
    }

    /**
     * Spot checks: nearly straight in front, and the wide point mirrored to the right side.
     */
    @Test
    public void angleOffCenterSpotChecks() {
        assertEquals(0.7, ShotSolver.solve(73.5 + OFFSET, 2.88 + OFFSET, MEASURED_CELL).angleOffCenterDegrees, ANGLE_EPSILON_DEG);
        ShotSolution mirroredWidePoint = ShotSolver.solve(119.75 + OFFSET, 6.34 + OFFSET, MEASURED_CELL);
        assertEquals(48.6, mirroredWidePoint.angleOffCenterDegrees, ANGLE_EPSILON_DEG);
        assertEquals(ShotSolution.Validity.ANGLE_TOO_WIDE, mirroredWidePoint.validity);
    }

    /**
     * Invalid positions report the right reason, and a good position is valid.
     */
    @Test
    public void validityReasons() {
        ShotSolution valid = ShotSolver.solve(73.5 + OFFSET, 2.88 + OFFSET, MEASURED_CELL); // ~44.7 in, straight in front
        assertEquals(ShotSolution.Validity.VALID, valid.validity);
        assertTrue(valid.inRange);
        assertEquals(ShotSolution.Validity.TOO_CLOSE, ShotSolver.solve(73.5 + OFFSET, 5.6 + OFFSET, MEASURED_CELL).validity); // ~42 in
        assertEquals(ShotSolution.Validity.TOO_FAR, ShotSolver.solve(73.5 + OFFSET, -20.0 + OFFSET, MEASURED_CELL).validity);  // ~67.6 in
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
