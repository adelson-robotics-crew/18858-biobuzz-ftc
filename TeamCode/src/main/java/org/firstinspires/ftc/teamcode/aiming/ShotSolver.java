package org.firstinspires.ftc.teamcode.aiming;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Works out the shot from the robot's position: distance and heading to the HIVE target point, the shooter
 * RPM from the shot table, and whether the position is a valid place to shoot from (distance range and angle
 * limit). Pure math, no hardware. All coordinates are our field frame (the frame the robot's pose reports).
 */
public final class ShotSolver {
    private static final ShotTable SHOT_TABLE = ShotTable.fromRobotConstants();

    // The robot is "straight in front of" the target when it's directly -y from it, i.e. at 270 deg around it
    private static final double STRAIGHT_IN_FRONT_DEGREES = 270.0;

    /**
     * Not meant to be instantiated; use the static solve() methods.
     */
    private ShotSolver() {
    }

    /**
     * Solves the shot from a robot pose. Only x and y matter; the robot's current heading doesn't change the shot.
     *
     * @param robotPose the robot's pose in our field frame (inches)
     * @return the shot solution for that position
     */
    public static ShotSolution solve(Pose robotPose) {
        return solve(robotPose.x(), robotPose.y());
    }

    /**
     * Solves the shot from a robot position.
     *
     * @param robotXInches robot x in our field frame
     * @param robotYInches robot y in our field frame
     * @return the shot solution for that position
     */
    public static ShotSolution solve(double robotXInches, double robotYInches) {
        double towardTargetX = RobotConstants.SHOT_TARGET_X_INCHES - robotXInches;
        double towardTargetY = RobotConstants.SHOT_TARGET_Y_INCHES - robotYInches;

        double distanceInches = Math.hypot(towardTargetX, towardTargetY);
        // The shooter fires out the back of the robot, so face directly away from the target (+ PI),
        // wrapped into [-PI, PI)
        double targetHeadingRadians = Angle.normalizeSigned(Math.atan2(towardTargetY, towardTargetX) + Math.PI);

        // Direction from the target to the robot, measured relative to straight in front (270 deg), wrapped to [-180, 180)
        double directionFromTargetDegrees = Math.toDegrees(Math.atan2(-towardTargetY, -towardTargetX));
        double angleOffCenterDegrees = Math.toDegrees(Angle.normalizeSigned(
                Math.toRadians(directionFromTargetDegrees - STRAIGHT_IN_FRONT_DEGREES)));

        ShotSolution.Validity validity;
        if (distanceInches < SHOT_TABLE.getMinInRangeInches()) {
            validity = ShotSolution.Validity.TOO_CLOSE;
        } else if (distanceInches > SHOT_TABLE.getMaxInRangeInches()) {
            validity = ShotSolution.Validity.TOO_FAR;
        } else if (Math.abs(angleOffCenterDegrees) > RobotConstants.MAX_ANGLE_OFF_CENTER_DEG) {
            validity = ShotSolution.Validity.ANGLE_TOO_WIDE;
        } else {
            validity = ShotSolution.Validity.VALID;
        }

        return new ShotSolution(distanceInches, targetHeadingRadians, SHOT_TABLE.rpmAt(distanceInches),
                angleOffCenterDegrees, validity);
    }
}
