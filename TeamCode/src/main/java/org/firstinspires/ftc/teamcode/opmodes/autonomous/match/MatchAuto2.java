package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;


/**
 * Match Auto 2, from the bottom wall: waits for the alliance partner to shoot, nudges off the wall, aims and shoots
 * (as if the TeleOp's aim trigger were held), turns back square, then strafes over and drives up the side to park.
 * Each drive leg finishes only once the robot is at its target pose (see Drivetrain's followPathCommand()).
 * Not listed on the Driver Station itself: RedMatchAuto2 and BlueMatchAuto2 choose the alliance.
 */
public abstract class MatchAuto2 extends MatchAutoBase {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // How long it sits still at the start before doing anything, so an alliance partner with its own auto can take
    // its shots first.
    private static final double AUTO_2_WAIT_FOR_PARTNER_SECONDS = 15.0;

    // Creates poses from (x, y, heading) with the heading given in degrees
    private static final PoseFactory POSE_FACTORY = PoseFactory.degrees();

    // Pedro coordinates, inches. On red, heading 270 deg = front of the robot faces -y (the bottom wall), so the
    // shooter (out the back) faces +y toward the red lower cell, about 50 in away: inside the shot table's range.
    // These are the RED side's poses; blue runs them rotated 180 deg about the field center (see forAlliance()).
    private static final Pose RED_START_POSE = POSE_FACTORY.of(38.889, 8.814, 270);
    // 2 in up the field (+y) off the wall, before turning to aim
    private static final Pose RED_SHOOT_POSE = POSE_FACTORY.of(38.889, 10.814, 270);
    // Strafe over toward the side wall, still facing 270 deg
    private static final Pose RED_SIDE_POSE = POSE_FACTORY.of(16.0, 10.814, 270);
    // Then straight up the side (+y) to park
    private static final Pose RED_PARKING_POSE = POSE_FACTORY.of(16.0, 94.45, 270);

    /**
     * The log file's name.
     *
     * @return "MatchAuto2"
     */
    @Override
    protected String routineName() {
        return "MatchAuto2";
    }

    /**
     * The red start pose, against the bottom wall facing 270 deg.
     *
     * @return the red start pose
     */
    @Override
    protected Pose redStartPose() {
        return RED_START_POSE;
    }

    /**
     * Wait, nudge 2 in off the wall, aim and shoot, turn back to the start heading, strafe over, drive up to park.
     *
     * @param allianceStartPose this alliance's start pose
     * @return the routine
     */
    @Override
    protected Command buildRoutine(Pose allianceStartPose) {
        Pose shootPose = forAlliance(RED_SHOOT_POSE);
        Pose sidePose = forAlliance(RED_SIDE_POSE);
        Pose parkingPose = forAlliance(RED_PARKING_POSE);
        return sequential(
                markStep("waitForPartner", allianceStartPose),
                instant(() -> robot.drivetrain.stop()), // sit still against the wall
                waitMs(AUTO_2_WAIT_FOR_PARTNER_SECONDS * 1000.0),
                markStep("nudgeOffWall", shootPose),
                robot.drivetrain.followPathCommand(straightPath(allianceStartPose, shootPose)),
                markStep("aimAndShoot", null), // the target heading comes from the shot, not a fixed pose
                aimAndShootCommand(),
                // Turn back to the start heading in place (the aim turned the robot toward the cell)
                markStep("turnBack", shootPose),
                robot.drivetrain.holdPoseCommand(shootPose),
                markStep("strafeToSide", sidePose),
                robot.drivetrain.followPathCommand(straightPath(shootPose, sidePose)),
                markStep("park", parkingPose),
                robot.drivetrain.followPathCommand(straightPath(sidePose, parkingPose))
        );
    }
}
