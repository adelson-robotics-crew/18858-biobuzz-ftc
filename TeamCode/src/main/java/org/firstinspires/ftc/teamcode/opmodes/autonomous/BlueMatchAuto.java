package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;

/**
 * Match Auto on the blue side. Only the alliance lives here; the routine is in MatchAuto, with every pose
 * rotated 180 deg about the field center from the red ones: start at (82.836, 8.097) facing 270 deg (front toward
 * the bottom wall, shooter toward the blue lower cell), pull away to (82.293, 26.023), then strafe to park at
 * (128.8, 23.58).
 */
@Autonomous(name = "Match Auto Blue")
public class BlueMatchAuto extends MatchAuto {

    /**
     * Runs on the blue side, with MatchAuto's red poses rotated 180 deg.
     *
     * @return Alliance.BLUE
     */
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
