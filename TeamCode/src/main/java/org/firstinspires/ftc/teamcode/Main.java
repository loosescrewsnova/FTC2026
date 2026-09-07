package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.SensorIMUOrthogonal;

@TeleOp(name = "Joystick Operations")
public class Main extends OpMode {

    private MecanumDrive mecanumDrive;
    private Intake intake;

    private SensorIMUOrthogonal imuOrthogonal;
    private IMU imu;

    private double endGameStart;
    private boolean isEndGame;

    @Override
    public void init() {

        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        isEndGame = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        endGameStart = getRuntime() + 90;
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        mecanumDrive.mecanumDrive(y, x, rx);
        mecanumDrive.updateOdometry(imu);
        intake.intake(gamepad1);

        if(getRuntime() >= endGameStart && !isEndGame) {
            gamepad1.rumbleBlips(3);
            isEndGame = true;
        }

        telemetry.addData("Front Left Power", mecanumDrive.getLeftFrontPower());
        telemetry.addData("Front Right Power", mecanumDrive.getRightFrontPower());
        telemetry.addData("Back Left Power", mecanumDrive.getLeftBackPower());
        telemetry.addData("Back Right Power", mecanumDrive.getRightBackPower());
        telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.addData("Is it Endgame", isEndGame);
        telemetry.update();
    }
}