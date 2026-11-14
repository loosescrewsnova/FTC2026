package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

public class Constants {

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
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("leftBackMotor");
        c.yPodName.set("rightFrontMotor");
        c.imuName.set("imu");
        c.xPodOffset.set(76.35208795703733);
        c.yPodOffset.set(-26.001351659920218);
        c.forwardTicksToInches.set(0.03744825256957815);
        c.strafeTicksToInches.set(3.0916447031977143);
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.REVERSE);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    });

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                (hardwareMap) -> new TwoWheelLocalizer(
                        hardwareMap,
                        Constants.localizerConfig
                ),
                (hardwareMap) -> new Mecanum(
                        hardwareMap,
                        Constants.drivetrainConfig
                )
        );
    }
}