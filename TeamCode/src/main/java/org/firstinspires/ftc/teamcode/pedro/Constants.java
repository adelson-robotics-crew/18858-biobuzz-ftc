package org.firstinspires.ftc.teamcode.pedro;
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




    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.922013891024853);
        c.yPodOffset.set(-1.3550159123938854);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

















//    public static MecanumConfig driveTrainConfig = new MecanumConfig(
//            c -> {
//                c.frontLeftName.set("front_left");
//                c.backLeftName.set("back_left");
//                c.frontRightName.set("front_right");
//                c.backRightName.set("back_right");
//
//                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
//                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
//            }
//    );
//
//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-2.9515901700718197);
//        c.yPodOffset.set(2.331910020723118);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });
//
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.3700126171262395);
//                Controller secondaryTranslationalForward = Controller.proportional(0.1367097751089259);
//                Controller primaryTranslationalLateral = Controller.proportional(0.36863354251765734);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.136200243890607);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.024019775997724452));
//                c.brake.set(Controller.proportionalFeedforward(0.020416809598065782));
//
//                c.headingFeedback.set(Controller.proportional(5.141697452239598));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04879450529382146, 0.004690341876366439));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.07300321840079235, 0.06082925074040077));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013730238437496707, 0.0015594692535384356));
//
//                c.maxAchievableForwardVelocity.set(50.279320954733585);
//                c.maxAchievableStrafeVelocity.set(40.80652659614686);
//                c.naturalForwardDeceleration.set(34.16730554163197);
//                c.naturalStrafeDeceleration.set(61.56747143283214);
//            }
//    );
//    public static Follower create(HardwareMap h) {
//        return new Follower(
//                new PinpointLocalizer(h, localizerConfig),
//                new Mecanum(h, driveTrainConfig),
//                new Foresight(foresightConfig)
//        );
//    }

}