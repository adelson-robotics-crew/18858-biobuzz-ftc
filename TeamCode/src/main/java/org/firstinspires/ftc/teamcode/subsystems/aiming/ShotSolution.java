package org.firstinspires.ftc.teamcode.subsystems.aiming;

/**
 * What ShotSolver worked out for one robot position: how far the target is, which way to face, how fast to
 * spin the shooter, and whether the position is a valid place to shoot from (and if not, why). Immutable.
 */
public final class ShotSolution {

    /**
     * Whether a position is valid to shoot from, and if not, the reason. Distance problems are reported
     * before angle problems when both apply.
     */
    public enum Validity {
        VALID("IN RANGE"),
        TOO_CLOSE("TOO CLOSE"),
        TOO_FAR("TOO FAR"),
        ANGLE_TOO_WIDE("ANGLE TOO WIDE");

        public final String label; // shown on telemetry

        /**
         * @param label the human-readable text shown in telemetry
         */
        Validity(String label) {
            this.label = label;
        }
    }

    public final double distanceInches;      // straight-line distance from the robot to the target
    public final double targetHeadingRadians; // heading that points the shooter (the robot's back) at the target, in [-PI, PI)
    public final double targetRpm;            // shooter RPM from the shot table for this distance
    public final double angleOffCenterDegrees; // 0 = straight -y from the target; negative = left side, positive = right side
    public final Validity validity;
    public final boolean inRange;             // true when validity is VALID

    /**
     * @param distanceInches        distance from the robot to the target
     * @param targetHeadingRadians  heading to face so the shooter points at the target
     * @param targetRpm             shooter RPM for this distance
     * @param angleOffCenterDegrees how far around the target the robot is from straight in front of it
     * @param validity              whether the position is valid, or why not
     */
    public ShotSolution(double distanceInches, double targetHeadingRadians, double targetRpm,
                        double angleOffCenterDegrees, Validity validity) {
        this.distanceInches = distanceInches;
        this.targetHeadingRadians = targetHeadingRadians;
        this.targetRpm = targetRpm;
        this.angleOffCenterDegrees = angleOffCenterDegrees;
        this.validity = validity;
        this.inRange = validity == Validity.VALID;
    }
}
