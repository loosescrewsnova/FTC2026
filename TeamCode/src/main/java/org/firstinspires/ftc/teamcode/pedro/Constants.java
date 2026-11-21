package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    private static final double POD_TICKS_PER_REV = 2000.0;
    private static final double POD_WHEEL_DIAMETER_INCHES = 1.2598;
    private static final double POD_INCHES_PER_TICK =
            Math.PI * POD_WHEEL_DIAMETER_INCHES / POD_TICKS_PER_REV;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFrontMotor");
        c.frontRightName.set("rightFrontMotor");
        c.backLeftName.set("leftBackMotor");
        c.backRightName.set("rightBackMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("forwardPod");
        c.yPodName.set("strafePod");
        c.imuName.set("imu");
        c.xPodOffset.set(4.618084400988725);
        c.yPodOffset.set(-0.14960021492894857);
        c.forwardTicksToInches.set(POD_INCHES_PER_TICK);
        c.strafeTicksToInches.set(POD_INCHES_PER_TICK);
        // Pedro's axes are +X forward and +Y left. The raw pod directions
        // used by MecanumDrive count forwardPod positive when moving forward
        // and strafePod positive when moving right, so only the strafe pod
        // needs to be reversed for Pedro's +Y-left convention.
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.REVERSE);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19240452191107);
                Controller secondaryTranslationalForward = Controller.proportional(0.07108832970262911);
                Controller primaryTranslationalLateral = Controller.proportional(2.6056104558411093);
                Controller secondaryTranslationalLateral = Controller.proportional(0.9627034402396409);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01711803500067648));
                c.brake.set(Controller.proportionalFeedforward(0.014550329750575007));

                c.headingFeedback.set(Controller.proportional(2.727781438327698));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06767466175370807, 1.53852931616305E-4));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09463597075477415, 0.08095039008654907));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.2705210416848615E-4, 9.182932982115346E-4));

                c.maxAchievableForwardVelocity.set(63.230084093063496);
                c.maxAchievableStrafeVelocity.set(53.44604411659418);
                c.naturalForwardDeceleration.set(53.06246661224255);
                c.naturalStrafeDeceleration.set(84.51371629156505);
            }
    );
}
