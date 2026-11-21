package org.firstinspires.ftc.teamcode.Mains;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.NoOdometryMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.PollenPusher;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.pollenpusherbutton;


/**
 * TeleOp with no Limelight and no odometry.
 *
 * gamepad1 controls:
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   L2 (hold)     = pollen pusher sweep: back swings 45 left, then 90 right.
 *                   Let go of L2 to stop.
 *
 * gamepad2 controls:
 *   Y             = shooter (hold)
 *   X             = feeder servo (hold)
 *   B             = intake toggle (click on, click again off)
 *   A             = outtake toggle (click on, click again off)
 */
@TeleOp(name = "pollenpushermain", group = "Mains")
public class pollenpushermain extends OpMode {


    // IMPORTANT: set these to match how the Control Hub is mounted on the robot.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


    private NoOdometryMecanum mecanum;
    private Shooter shooter;
    private Intake intake;
    private Feeder feeder;
    private PollenPusher pollenPusher;
    private pollenpusherbutton pusherButton;
    private IMU imu;


    // Runs once when you press INIT.
    @Override
    public void init() {


        mecanum = new NoOdometryMecanum(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        pollenPusher = new PollenPusher(hardwareMap);


        // Control Hub's built-in IMU. "imu" is the default name in the robot config.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));
        imu.resetYaw();


        telemetry.addLine("Initialized - press START");
        telemetry.update();
    }


    // Runs once when you press START.
    @Override
    public void start() {
        imu.resetYaw();
    }


    // Runs over and over after START.
    @Override
    public void loop() {


        // Yaw in radians, counterclockwise-positive.
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        // The sweep runs first. If it is active, it owns the wheels.
        pusherButton.update(gamepad1.left_trigger, heading);


        if (!pusherButton.isBusy()) {
            mecanum.drive(gamepad1);
        }


        shooter.shooter(gamepad2);
        intake.intake(gamepad2);
        feeder.feeder(gamepad2);
        pollenPusher.pollenPusher(gamepad2);


        telemetry.addData("Sweep state", pusherButton.getStateName());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Shooter power", shooter.getShooterPower());
        telemetry.addData("Intake power", Intake.getIntakePower());
        telemetry.addData("Outtake power", Intake.getOuttakePower());
        telemetry.addData("Feeder power", Feeder.getFeedPower());
        telemetry.update();
    }


    // Runs once when you press STOP.
    @Override
    public void stop() {
        mecanum.stop();
        pusherButton.cancel();
        shooter.stop();
        intake.stop();
        feeder.stop();
        pollenPusher.stop();
    }
}

