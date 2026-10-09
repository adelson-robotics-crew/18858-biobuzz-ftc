package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Match Auto 1 on the red side. Only the alliance lives here; the routine is in MatchAuto1.
 */
@Autonomous(name = "Match Auto 1 Red")
public class RedMatchAuto1 extends MatchAuto1 {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
