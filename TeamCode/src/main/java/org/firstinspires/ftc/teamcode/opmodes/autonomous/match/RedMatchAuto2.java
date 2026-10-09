package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Match Auto 2 on the red side. Only the alliance lives here; the routine is in MatchAuto2.
 */
@Autonomous(name = "Match Auto 2 Red")
public class RedMatchAuto2 extends MatchAuto2 {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
