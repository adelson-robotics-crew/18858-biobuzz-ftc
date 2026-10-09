package org.firstinspires.ftc.teamcode.opmodes.autonomous.match;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Match Auto 1 on the blue side. Only the alliance lives here; the routine is in MatchAuto1, with every pose
 * rotated 180 deg about the field center from the red ones: start at (82.836, 8.097) facing 270 deg (front toward
 * the bottom wall, shooter toward the blue lower cell), pull away to (82.293, 26.023), then strafe to park at
 * (128.8, 23.58).
 */
@Autonomous(name = "Match Auto 1 Blue")
public class BlueMatchAuto1 extends MatchAuto1 {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
