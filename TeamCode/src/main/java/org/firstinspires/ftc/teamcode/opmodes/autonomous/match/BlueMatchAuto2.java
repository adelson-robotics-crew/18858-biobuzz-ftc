package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Match Auto 2 on the blue side. Only the alliance lives here; the routine is in MatchAuto2, with every pose
 * rotated 180 deg about the field center from the red ones: start at (102.611, 132.686) facing 90 deg against the
 * top wall, nudge to (102.611, 130.686), aim at the blue upper cell, strafe to (127.5, 130.686), then drive down to
 * park at (127.5, 47.05).
 */
@Autonomous(name = "Match Auto 2 Blue")
public class BlueMatchAuto2 extends MatchAuto2 {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
