package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

/**
 * Match Auto 1: starts with 4 balls and shoots them from the starting pose with the same aim-and-shoot as the
 * TeleOp's aim trigger (see aimAndShootCommand()), then pulls away from the wall and strafes into the parking
 * square. The start heading already faces the cell, so aiming barely turns it (90 deg on red, 270 deg on blue). Each
 * drive leg finishes only once the robot is at its target pose (see Drivetrain's followPathCommand()).
 * Not listed on the Driver Station itself: RedMatchAuto1 and BlueMatchAuto1 choose the alliance.
 */
public abstract class MatchAuto1 extends MatchAuto {
    // Creates poses from (x, y, heading) with the heading given in degrees
    private static final PoseFactory POSE_FACTORY = PoseFactory.degrees();

    // Pedro coordinates, inches. On red, heading 90 deg = front of the robot faces +y (toward the wall behind the
    // start), so the shooter (out the back) faces -y toward the red upper cell. The heading never changes.
    // These are the RED side's poses; blue runs them rotated 180 deg about the field center (see forAlliance()):
    // blue starts at the bottom of the field facing 270 deg, and every move is turned around to match.
    private static final Pose RED_START_POSE = POSE_FACTORY.of(58.664, 133.403, 90);      // also where it shoots from
    private static final Pose RED_LEAVE_WALL_POSE = POSE_FACTORY.of(59.207, 115.477, 90); // pull off the wall (~18 in)
    private static final Pose RED_PARKING_POSE = POSE_FACTORY.of(12.7, 117.92, 90);       // strafe into parking

    /**
     * The log file's name.
     *
     * @return "MatchAuto1"
     */
    @Override
    protected String routineName() {
        return "MatchAuto1";
    }

    /**
     * The red start pose, against the top wall facing 90 deg.
     *
     * @return the red start pose
     */
    @Override
    protected Pose redStartPose() {
        return RED_START_POSE;
    }

    /**
     * Shoot from the start, pull off the wall, then strafe to park.
     *
     * @param allianceStartPose this alliance's start pose
     * @return the routine
     */
    @Override
    protected Command buildRoutine(Pose allianceStartPose) {
        Pose leaveWallPose = forAlliance(RED_LEAVE_WALL_POSE);
        Pose parkingPose = forAlliance(RED_PARKING_POSE);
        return sequential(
                markStep("aimAndShoot", null), // the target heading comes from the shot, not a fixed pose
                aimAndShootCommand(),
                markStep("leaveWall", leaveWallPose),
                robot.drivetrain.followPathCommand(straightPath(allianceStartPose, leaveWallPose)),
                markStep("park", parkingPose),
                robot.drivetrain.followPathCommand(straightPath(leaveWallPose, parkingPose))
        );
    }
}
