package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Joystick Operations")
public class JoystickOperations extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        if (leftPower > 1.0) {
            leftPower = 1.0;
        }

        if (leftPower < -1.0) {
            leftPower = -1.0;
        }

        if (rightPower > 1.0) {
            rightPower = 1.0;
        }

        if (rightPower < -1.0) {
            rightPower = -1.0;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}