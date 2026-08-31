package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private double leftPower;
    private double rightPower;

    public Drive(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(Gamepad gamepad) {

        leftPower = gamepad.left_stick_y;
        rightPower = gamepad.right_stick_y;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if(leftPower > 0.8) {
            leftPower = 0.8;
        }

        if(leftPower < -0.8) {
            leftPower = -0.8;
        }

        if(rightPower > 0.8) {
            rightPower = 0.8;
        }

        if(rightPower < -0.8) {
            rightPower = -0.8;
        }
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }
}