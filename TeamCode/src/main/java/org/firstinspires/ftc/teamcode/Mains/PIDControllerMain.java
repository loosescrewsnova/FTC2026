package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.IMUOrthogonalNew;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.NoOdometryMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.PollenPusher;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name = "OnePlayer PID Turret", group = "Mains")
public class PIDControllerMain extends LinearOpMode {

    private static final double TURRET_MAX_POWER = 0.5;

    @Override
    public void runOpMode() {

        NoOdometryMecanum mecanum = new NoOdometryMecanum(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        PollenPusher pollenPusher = new PollenPusher(hardwareMap);

        IMUOrthogonalNew imu = new IMUOrthogonalNew(hardwareMap);
        LimelightLocalizer limelightLocalizer = new LimelightLocalizer(hardwareMap);
        PIDController turretPID = new PIDController();

        telemetry.addLine("Initialized - press START");
        telemetry.update();

        waitForStart();
        limelightLocalizer.start();

        while (opModeIsActive()) {

            limelightLocalizer.update(imu.getYawDegrees());

            boolean turretRequested = gamepad1.right_bumper;
            boolean hasTarget = limelightLocalizer.hasValidTarget();
            boolean turretActive = turretRequested && hasTarget;

            if (turretActive) {
                double txErrorRadians = Math.toRadians(limelightLocalizer.getTx());
                double turretPower = turretPID.calculate(txErrorRadians);
                turretPower = Math.max(
                        -TURRET_MAX_POWER,
                        Math.min(TURRET_MAX_POWER, turretPower)
                );

                // Replace only the drive's turn component with PID output.
                mecanum.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turretPower);
            } else {
                turretPID.reset();
                mecanum.drive(gamepad1);
            }

            // Keep the OnePlayer mechanism controls and their button mappings.
            shooter.shooter(gamepad1);
            intake.intake(gamepad1);
            feeder.feeder(gamepad1);
            pollenPusher.pollenPusher(gamepad1);

            telemetry.addData("Shooter power", shooter.getShooterPower());
            telemetry.addData("Intake power", Intake.getIntakePower());
            telemetry.addData("Outtake power", Intake.getOuttakePower());
            telemetry.addData("Feeder power", Feeder.getFeedPower());
            telemetry.addData("Vision / Turret", "%s / %s",
                    hasTarget ? "TAG" : "none", turretActive ? "ON" : "off");
            telemetry.update();
        }

        mecanum.stop();
        shooter.stop();
        intake.stop();
        feeder.stop();
        pollenPusher.stop();
    }
}
