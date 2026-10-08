package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;

/**
 * The practice TeleOp: the same as the match TeleOp, but starting from the corner the shot measurements were
 * taken from and aiming at the blue hive (the measured cell is the blue lower one, on B). It always starts from
 * that corner, never from a carried-over pose.
 * Its coordinates are the practice frame: (0, 0) is the robot in the practice corner (Pedro (8.5, 8.5)), which is
 * what telemetry shows. The hive cells are shifted into that frame, so aiming is unchanged. The reset button (Back)
 * resets the whole pose to (0, 0, 0 deg) here, not just the heading.
 */
@TeleOp(name = "DriveTest")
public class DriveTest extends MatchTeleOp {

    /**
     * Always aims at the blue hive, where the shot measurements were taken.
     *
     * @return Alliance.BLUE
     */
    @Override
    protected Alliance fixedAlliance() {
        return Alliance.BLUE;
    }

    /**
     * The robot placed in the practice corner, facing +x (away from the driver): (0, 0, 0 deg) in the practice frame.
     *
     * @return the practice starting pose
     */
    @Override
    protected Pose fallbackStartPose() {
        return PoseFactory.degrees().of(0, 0, 0);
    }

    /**
     * The practice frame's (0, 0) is Pedro (8.5, 8.5), the frame the shot measurements were taken in.
     *
     * @return 8.5 inches
     */
    @Override
    protected double coordinateOffsetInches() {
        return 8.5;
    }

    /**
     * Back resets the whole pose to (0, 0, 0 deg), so the robot can be put back in the practice corner and re-zeroed.
     *
     * @return true
     */
    @Override
    protected boolean resetButtonResetsPosition() {
        return true;
    }

    /**
     * Always starts from the practice corner, even right after another OpMode.
     *
     * @return false
     */
    @Override
    protected boolean useCarriedPose() {
        return false;
    }

    /**
     * The practice frame starts at 0 deg facing away from the driver, so the sticks stay as they are even though
     * this TeleOp aims at the blue hive.
     *
     * @return false
     */
    @Override
    protected boolean driverSideFollowsAlliance() {
        return false;
    }
}
