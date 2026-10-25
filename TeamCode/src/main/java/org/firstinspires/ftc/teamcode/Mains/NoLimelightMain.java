package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.NoOdometryMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.PollenPusher;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

/**
 * TeleOp with no Limelight and no odometry.
 *
 * gamepad1 controls:
 *   Left stick    = drive / strafe
 *   Right stick X = turn
 *   Y (top)       = shooter (hold)
 *   X (left)      = feeder servo (hold)
 *   B (right)     = intake toggle (click on, click again off)
 *   A (bottom)    = pollen pusher (hold)
 */
@TeleOp(name = "NoLimelightMain", group = "Mains")
public class NoLimelightMain extends LinearOpMode {

    @Override
    public void runOpMode() {

        NoOdometryMecanum mecanum = new NoOdometryMecanum(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        PollenPusher pollenPusher = new PollenPusher(hardwareMap);

        telemetry.addLine("Initialized - press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            mecanum.driveWithGamepad(gamepad1);
            shooter.shooter(gamepad1);
            intake.intake(gamepad1);
            feeder.feeder(gamepad1);
            pollenPusher.pollenPusher(gamepad1);

            telemetry.addData("Shooter power", shooter.getShooterPower());
            telemetry.addData("Intake power", intake.getIntakePower());
            telemetry.addData("Feeder power", Feeder.getFeedPower());
            telemetry.update();
        }

        mecanum.stop();
        shooter.stop();
        intake.stop();
        feeder.stop();
        pollenPusher.stop();
    }
}