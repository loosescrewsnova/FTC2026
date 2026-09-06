package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Joystick Operations")
public class TankDriveMain extends OpMode {

    private Drive drive;
    private Intake intake;

    private double endGameStart;
    private boolean isEndGame;

    @Override
    public void init() {

        drive = new Drive(hardwareMap);
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

        drive.drive(gamepad1);
        intake.intake(gamepad1);

        if(getRuntime() >= endGameStart && !isEndGame) {
            gamepad1.rumbleBlips(3);
            isEndGame = true;
        }

        telemetry.addData("Right Motor Power", drive.getRightPower());
        telemetry.addData("Left motor Power", drive.getLeftPower());
        telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.addData("Is it Endgame", isEndGame);
        telemetry.update();
    }
}