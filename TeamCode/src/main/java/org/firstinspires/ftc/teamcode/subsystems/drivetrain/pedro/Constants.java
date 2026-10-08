package org.firstinspires.ftc.teamcode.subsystems.drivetrain.pedro;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig driveTrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("front_left");
                c.backLeftName.set("back_left");
                c.frontRightName.set("front_right");
                c.backRightName.set("back_right");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-4.830174784022054);
//        c.yPodOffset.set(-8.953716848778914);
        c.xPodOffset.set(6.5); // manually measured
        c.yPodOffset.set(-2.5); // manually measured
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2171153015657996);
                Controller secondaryTranslationalForward = Controller.proportional(0.08021830250085868);
                Controller primaryTranslationalLateral = Controller.proportional(0.27364403801406606);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1011041601428364);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01557896158374678));
                c.brake.set(Controller.proportionalFeedforward(0.013242117346184763));

                c.headingFeedback.set(Controller.proportional(2.8550869867835003));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0525612605700482, 0.0041551460282305665));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07462302829865798, 0.04665132143355067));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013349621662311034, 0.0015758376726539747));

                c.maxAchievableForwardVelocity.set(67.43253489798936);
                c.maxAchievableStrafeVelocity.set(58.18988835856642);
                c.naturalForwardDeceleration.set(32.97033091134326);
                c.naturalStrafeDeceleration.set(62.728074657578);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveTrainConfig),
                new Foresight(foresightConfig)
        );
    }
}