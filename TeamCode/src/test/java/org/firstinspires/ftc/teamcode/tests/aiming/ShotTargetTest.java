package org.firstinspires.ftc.teamcode.tests.aiming;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTarget;
import org.junit.Test;

/**
 * Tests that the three derived HIVE cells land where they were worked out on the field, and that a shot at any
 * cell from the matching symmetric position is the same shot as at the measured cell.
 */
public class ShotTargetTest {
    private static final double POSITION_EPSILON_INCHES = 1e-9;
    private static final double ANGLE_EPSILON_DEG = 1e-9;

    /**
     * Each cell is where it was worked out by hand: blue lower (measured), blue upper, red upper, red lower.
     */
    @Test
    public void cellsMatchHandWorkedCoordinates() {
        assertCell(ShotTarget.BLUE_LOWER, 81.45, 56.06, 270.0);
        assertCell(ShotTarget.BLUE_UPPER, 81.45, 85.44, 90.0);
        assertCell(ShotTarget.RED_UPPER, 60.05, 85.44, 90.0);
        assertCell(ShotTarget.RED_LOWER, 60.05, 56.06, 270.0);
    }

    /**
     * Every measured shot, moved to the symmetric spot for each derived cell, gets the same distance, RPM, and
     * validity as the original, and a shooting heading moved by the same symmetry.
     */
    @Test
    public void measuredShotsCarryOverToEveryCell() {
        double centerX = RobotConstants.FIELD_SYMMETRY_CENTER_X_INCHES;
        double centerY = RobotConstants.FIELD_SYMMETRY_CENTER_Y_INCHES;
        for (double[] row : ShotMeasurements.ROWS) {
            double robotX = ShotMeasurements.pedroX(row);
            double robotY = ShotMeasurements.pedroY(row);
            ShotSolution measuredShot = ShotSolver.solve(robotX, robotY, ShotTarget.BLUE_LOWER);
            double measuredHeadingDeg = Math.toDegrees(measuredShot.targetHeadingRadians);

            // Blue upper: flipped over the horizontal center line, so the heading is negated
            assertSameShot(row, measuredShot, -measuredHeadingDeg,
                    ShotSolver.solve(robotX, 2 * centerY - robotY, ShotTarget.BLUE_UPPER));
            // Red upper: rotated 180 deg about the center, so the heading turns by 180 deg
            assertSameShot(row, measuredShot, measuredHeadingDeg + 180.0,
                    ShotSolver.solve(2 * centerX - robotX, 2 * centerY - robotY, ShotTarget.RED_UPPER));
            // Red lower: red upper flipped over the horizontal center line, i.e. the measured shot flipped over the
            // vertical center line, so the heading becomes 180 deg minus itself
            assertSameShot(row, measuredShot, 180.0 - measuredHeadingDeg,
                    ShotSolver.solve(2 * centerX - robotX, robotY, ShotTarget.RED_LOWER));
        }
    }

    /**
     * Checks a cell's position and front direction.
     *
     * @param cell                          the cell to check
     * @param expectedX                     expected Pedro x
     * @param expectedY                     expected Pedro y
     * @param expectedFrontDirectionDegrees expected front direction, in [0, 360)
     */
    private static void assertCell(ShotTarget cell, double expectedX, double expectedY,
                                   double expectedFrontDirectionDegrees) {
        assertEquals(cell.name + " x", expectedX, cell.xInches, POSITION_EPSILON_INCHES);
        assertEquals(cell.name + " y", expectedY, cell.yInches, POSITION_EPSILON_INCHES);
        assertEquals(cell.name + " front direction", expectedFrontDirectionDegrees, cell.frontDirectionDegrees,
                ANGLE_EPSILON_DEG);
    }

    /**
     * Checks that a mirrored shot matches the measured shot: same distance, RPM, and validity, and the expected
     * shooting heading. The angle off center can flip sign under a mirror, so only its size is compared.
     *
     * @param row                 the measurement row, for messages
     * @param measuredShot        the shot at the measured cell
     * @param expectedHeadingDeg  the heading the mirrored shot should have, in degrees (any wrap)
     * @param mirroredShot        the shot at the derived cell from the mirrored position
     */
    private static void assertSameShot(double[] row, ShotSolution measuredShot, double expectedHeadingDeg,
                                       ShotSolution mirroredShot) {
        String message = ShotMeasurements.describe(row);
        assertEquals(message, measuredShot.distanceInches, mirroredShot.distanceInches, POSITION_EPSILON_INCHES);
        assertEquals(message, measuredShot.targetRpm, mirroredShot.targetRpm, 1e-6);
        assertEquals(message, measuredShot.validity, mirroredShot.validity);
        assertEquals(message, Math.abs(measuredShot.angleOffCenterDegrees),
                Math.abs(mirroredShot.angleOffCenterDegrees), 1e-6);
        double headingDifferenceDeg = wrapDegrees(Math.toDegrees(mirroredShot.targetHeadingRadians) - expectedHeadingDeg);
        assertEquals(message + " heading", 0.0, headingDifferenceDeg, 1e-6);
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
