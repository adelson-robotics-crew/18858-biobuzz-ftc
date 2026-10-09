package org.firstinspires.ftc.teamcode.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolution;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.aiming.ShotTarget;
import org.junit.Test;

/**
 * Tests turning red-side poses into blue ones (a 180 deg rotation about the field center), and that the aim math
 * agrees with the way each auto starts.
 */
public class AllianceTest {
    private static final double POSITION_EPSILON_INCHES = 1e-9;
    private static final double ANGLE_EPSILON_DEG = 1e-9;

    /**
     * Match Auto's red poses rotate to the expected blue ones: blue starts at the bottom of the field facing 270 deg
     * (front toward the bottom wall), pulls away upward, and strafes toward +x to park.
     */
    @Test
    public void matchAutoPosesRotateToBlue() {
        assertPose(82.836, 8.097, 270.0, Alliance.BLUE.fromRedSide(redPose(58.664, 133.403, 90)));
        assertPose(82.293, 26.023, 270.0, Alliance.BLUE.fromRedSide(redPose(59.207, 115.477, 90)));
        assertPose(128.8, 23.58, 270.0, Alliance.BLUE.fromRedSide(redPose(12.7, 117.92, 90)));
    }

    /**
     * Every move turns around with the rotation: red's strafe toward -x to park becomes blue's strafe toward +x,
     * and red's pull away from the top wall (-y) becomes blue's pull away from the bottom wall (+y).
     */
    @Test
    public void movesTurnAroundForBlue() {
        Pose redLeaveWall = redPose(59.207, 115.477, 90);
        Pose redParking = redPose(12.7, 117.92, 90);
        Pose blueLeaveWall = Alliance.BLUE.fromRedSide(redLeaveWall);
        Pose blueParking = Alliance.BLUE.fromRedSide(redParking);
        assertEquals(-(redParking.x() - redLeaveWall.x()), blueParking.x() - blueLeaveWall.x(), POSITION_EPSILON_INCHES);
        assertEquals(-(redParking.y() - redLeaveWall.y()), blueParking.y() - blueLeaveWall.y(), POSITION_EPSILON_INCHES);
    }

    /**
     * Headings turn around: facing +x (0 deg) becomes facing -x (180 deg), and 45 deg becomes 225 deg.
     */
    @Test
    public void headingsTurnAround() {
        assertPose(141.5, 141.5, 180.0, Alliance.BLUE.fromRedSide(redPose(0.0, 0.0, 0)));
        assertPose(141.5, 141.5, 225.0, Alliance.BLUE.fromRedSide(redPose(0.0, 0.0, 45)));
    }

    /**
     * From each auto's starting spot (where it shoots at a fixed RPM), aiming at the cell that auto shoots into
     * gives exactly the heading the robot starts at, from the distance the shot table expects (~48 in), straight in
     * front of the cell. Ties the auto's start heading, the cell positions, and the aim math together: if any of
     * them pointed the wrong way, this would fail.
     */
    @Test
    public void aimFromEachAutoStartMatchesTheStartHeading() {
        Pose redStart = redPose(58.664, 133.403, 90);
        Pose blueStart = Alliance.BLUE.fromRedSide(redStart);

        // Red shoots its upper cell and blue its lower cell: on each side, the cell on that driver's left (X)
        assertAimMatchesStart(redStart, Alliance.RED.leftCell);
        assertAimMatchesStart(blueStart, Alliance.BLUE.leftCell);
    }

    /**
     * Red poses pass through unchanged.
     */
    @Test
    public void redPosesAreUnchanged() {
        Pose redParking = redPose(12.7, 117.92, 90);
        assertSame(redParking, Alliance.RED.fromRedSide(redParking));
    }


    /**
     * Checks that aiming at a cell from a start pose asks for the start pose's own heading, and that the start is
     * a valid shot: in range and straight in front of the cell.
     *
     * @param startPose the auto's starting pose
     * @param cell      the cell that auto shoots into
     */
    private static void assertAimMatchesStart(Pose startPose, ShotTarget cell) {
        ShotSolution shot = ShotSolver.solve(startPose, cell);
        assertEquals(cell.name + " validity", ShotSolution.Validity.VALID, shot.validity);
        assertEquals(cell.name + " distance", 47.98, shot.distanceInches, 0.01);
        // The start is almost exactly in line with the cell; 1.7 deg off is the 1.4 in sideways offset at 48 in
        assertEquals(cell.name + " angle off center", 0.0, shot.angleOffCenterDegrees, 2.0);
        assertPose(startPose.x(), startPose.y(), Math.toDegrees(shot.targetHeadingRadians),
                new Pose(startPose.x(), startPose.y(), startPose.heading()), 2.0);
    }

    /**
     * Builds a pose from a heading in degrees.
     *
     * @param xInches        Pedro x
     * @param yInches        Pedro y
     * @param headingDegrees heading, in degrees
     * @return the pose
     */
    private static Pose redPose(double xInches, double yInches, double headingDegrees) {
        return new Pose(xInches, yInches, Math.toRadians(headingDegrees));
    }

    /**
     * Checks a pose's position and heading, comparing headings with wrap-around (so 360 deg equals 0 deg).
     *
     * @param expectedX              expected Pedro x
     * @param expectedY              expected Pedro y
     * @param expectedHeadingDegrees expected heading, in degrees
     * @param actualPose             the pose to check
     */
    private static void assertPose(double expectedX, double expectedY, double expectedHeadingDegrees, Pose actualPose) {
        assertPose(expectedX, expectedY, expectedHeadingDegrees, actualPose, ANGLE_EPSILON_DEG);
    }

    /**
     * Checks a pose's position and heading, with a chosen heading tolerance.
     *
     * @param expectedX              expected Pedro x
     * @param expectedY              expected Pedro y
     * @param expectedHeadingDegrees expected heading, in degrees
     * @param actualPose             the pose to check
     * @param headingToleranceDeg    how far off the heading may be, in degrees
     */
    private static void assertPose(double expectedX, double expectedY, double expectedHeadingDegrees, Pose actualPose,
                                   double headingToleranceDeg) {
        assertEquals(expectedX, actualPose.x(), POSITION_EPSILON_INCHES);
        assertEquals(expectedY, actualPose.y(), POSITION_EPSILON_INCHES);
        // Double modulo wraps the difference into [-180, 180) before comparing it to 0
        double headingDifferenceDegrees = ((Math.toDegrees(actualPose.heading()) - expectedHeadingDegrees + 180.0)
                % 360.0 + 360.0) % 360.0 - 180.0;
        assertEquals(0.0, headingDifferenceDegrees, headingToleranceDeg);
    }
}
