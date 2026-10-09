package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;

/**
 * The TeleOp to run after the autonomous in a match: starts from where the auto left the robot and aims at the
 * hive of the auto that ran (Match Auto Red or Match Auto Blue). If no auto has run since the robot was restarted,
 * it can't tell the alliance, so the right trigger (aim and shoot) stays off; run Match Auto Red or Match Auto Blue
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
