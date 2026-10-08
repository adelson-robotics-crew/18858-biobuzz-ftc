package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.aiming.Alliance;

/**
 * Match Auto on the red side. Only the alliance lives here; the routine is in MatchAuto.
 */
@Autonomous(name = "Match Auto Red")
public class RedMatchAuto extends MatchAuto {

    /**
     * Runs on the red side, with MatchAuto's poses as written.
     *
     * @return Alliance.RED
     */
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
