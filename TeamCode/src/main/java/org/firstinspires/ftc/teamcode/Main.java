package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Joystick Operations")
public class Main extends OpMode {

    private Drive drive;
    private Intake intake;

    @Override
    public void init() {

        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        drive.drive(gamepad1);
        intake.intake(gamepad1);

        telemetry.addData("Left Power", drive.getLeftPower());
        telemetry.addData("Right Power", drive.getRightPower());
        telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.update();
    }
}