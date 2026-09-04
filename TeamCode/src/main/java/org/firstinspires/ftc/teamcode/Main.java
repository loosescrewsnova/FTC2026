package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Joystick Operations")
public class Main extends OpMode {

    private MecanumDrive mecanumDrive;
    private Intake intake;

    @Override
    public void init() {

        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        mecanumDrive.mecunamDrive(y, x, rx);
        intake.intake(gamepad1);

        telemetry.addData("Front Left Power", mecanumDrive.getLeftFrontPower());
        telemetry.addData("Front Right Power", mecanumDrive.getRightFrontPower());
        telemetry.addData("Back Left Power", mecanumDrive.getLeftBackPower());
        telemetry.addData("Back Right Power", mecanumDrive.getRightBackPower());
        telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.update();
    }
}