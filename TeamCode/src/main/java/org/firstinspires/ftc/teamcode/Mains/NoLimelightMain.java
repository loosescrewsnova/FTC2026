package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.NoOdometryMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.shooter.Shooter;

@TeleOp(name = "First Starter Bot Code3")
public class NoLimelightMain extends OpMode {

    private NoOdometryMecanum noOdometryMecanum;
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;

    private double endGameStart;
    private boolean isEndGame;

    @Override
    public void init() {

        noOdometryMecanum =
                new NoOdometryMecanum(hardwareMap);

        intake = new Intake(hardwareMap);

        feeder = new Feeder(hardwareMap);

        shooter = new Shooter(hardwareMap);

        isEndGame = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        // Endgame begins 90 seconds after TeleOp starts
        endGameStart = getRuntime() + 90;
    }

    @Override
    public void loop() {

        // ----------------------------
        // INTAKE, FEEDER, AND SHOOTER
        // ----------------------------

        // Hold left trigger to run intake
        intake.intake(gamepad1);

        // Hold right trigger to operate feeder
        feeder.feeder(gamepad1);

        // Shooter controls are handled by the Shooter subsystem
        shooter.shooter(gamepad1);

        // ----------------------------
        // MECANUM DRIVE
        // ----------------------------

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        noOdometryMecanum.mecanumDrive(y, x, rx);

        // ----------------------------
        // ENDGAME RUMBLE
        // ----------------------------

        if (getRuntime() >= endGameStart && !isEndGame) {

            gamepad1.rumbleBlips(3);

            isEndGame = true;
        }

        // ----------------------------
        // TELEMETRY
        // ----------------------------

        telemetry.addData(
                "Intake / Feeder / Shooter",
                "%.2f / %.2f / %.2f",
                intake.getIntakePower(),
                feeder.getFeederPosition(),
                shooter.getShooterSpeed()
        );

        telemetry.addData("Endgame", isEndGame);

        telemetry.addData(
                "Drive LF / RF",
                "%.2f / %.2f",
                noOdometryMecanum.getLeftFrontPower(),
                noOdometryMecanum.getRightFrontPower()
        );

        telemetry.update();
    }
}
