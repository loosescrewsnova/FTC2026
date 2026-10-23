package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.NoOdometryMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

/**
 * TeleOp with no Limelight and no odometry.
 *
 * gamepad1 controls:
 *   Left stick          = drive / strafe
 *   Right stick X       = turn
 *   D-pad right (hold)  = shooter speed up   (0.2 per second, starts at 0)
 *   D-pad left (hold)   = shooter speed down (0.2 per second, never below 0)
 *   Right trigger (hold) = intake
 *   Left trigger (hold)  = feeder
 */
@TeleOp(name = "NoLimelightMain", group = "Mains")
public class NoLimelightMain extends LinearOpMode {

    @Override
    public void runOpMode() {

        NoOdometryMecanum mecanum = new NoOdometryMecanum(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);

        telemetry.addLine("Initialized - press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            mecanum.driveWithGamepad(gamepad1);
            shooter.shooter(gamepad1);
            intake.intake(gamepad1);
            feeder.feeder(gamepad1);

            telemetry.addData("Intake power", "%.2f", intake.getIntakePower());
            telemetry.update();
        }

        mecanum.stop();
        shooter.stop();
        intake.stop();
    }
}