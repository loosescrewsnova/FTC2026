package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
// ADAPT: fix this import to wherever your IMUOrthogonalNew class lives.
import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
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
public class pollenpushermain extends LinearOpMode {

    @Override
    public void runOpMode() {

        NoOdometryMecanum mecanum = new NoOdometryMecanum(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        PollenPusher pollenPusher = new PollenPusher(hardwareMap);
        pollenpusherbutton pusherButton = new pollenpusherbutton(hardwareMap);

        IMUOrthogonalNew imu = new IMUOrthogonalNew(hardwareMap);

        telemetry.addLine("Initialized - press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pusherButton.update(gamepad1.left_trigger, imu.getYawRadians());

            if (!pusherButton.isBusy()) {
                mecanum.drive(gamepad1);
            }

            shooter.shooter(gamepad2);
            intake.intake(gamepad2);
            feeder.feeder(gamepad2);
            pollenPusher.pollenPusher(gamepad2);

            telemetry.addData("Sweep state", pusherButton.getStateName());
            telemetry.addData("Shooter power", shooter.getShooterPower());
            telemetry.addData("Intake power", Intake.getIntakePower());
            telemetry.addData("Outtake power", Intake.getOuttakePower());
            telemetry.addData("Feeder power", Feeder.getFeedPower());
            telemetry.update();
        }

        mecanum.stop();
        pusherButton.cancel();
        shooter.stop();
        intake.stop();
        feeder.stop();
        pollenPusher.stop();
    }
}