package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
/** This routine does not show up in the DS - it's a wrapper to create the teleop routine
 *
 *
 */

/**
 * The TeleOp to run after the autonomous in a match: starts from where the auto left the robot and aims at the
 * hive of the auto that ran (any Match Auto, red or blue). If no auto has run since the robot was restarted,
 * it can't tell the alliance, so the right trigger (aim and shoot) stays off; run a red or blue Match Auto
 * first.
 */
@TeleOp(name = "Match TeleOp")
public class AutoAllianceMatchTeleOp extends MatchTeleOp {

    /**
     * No fixed alliance: it comes from the auto that ran (Robot.getSavedAlliance()).
     *
     * @return null
     */
    @Override
    protected Alliance fixedAlliance() {
        return null;
    }

    /**
     * No fallback pose: without the auto's pose the robot's position isn't known.
     *
     * @return null
     */
    @Override
    protected Pose fallbackStartPose() {
        return null;
    }
}
