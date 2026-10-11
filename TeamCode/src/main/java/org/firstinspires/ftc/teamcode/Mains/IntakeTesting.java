package org.firstinspires.ftc.teamcode.Mains;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Testing")
public class IntakeTesting extends OpMode {

    private DcMotor intakeMotor;
    private DcMotor feederMotor;

    @Override
    public void init() {
        // Initialize motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        // Start with both motors stopped
        intakeMotor.setPower(0);
        feederMotor.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Right bumper: Run intake
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0);
        }

        // Left bumper: Run feeder
        if (gamepad1.left_bumper) {
            feederMotor.setPower(1.0);
        } else {
            feederMotor.setPower(0);
        }

        // Display motor powers
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Feeder Power", feederMotor.getPower());
        telemetry.addData("Right Bumper", "Intake");
        telemetry.addData("Left Bumper", "Feeder");
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
        feederMotor.setPower(0);
    }
}