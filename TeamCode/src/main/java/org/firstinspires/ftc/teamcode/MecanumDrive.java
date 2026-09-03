package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

    private final DcMotor rightFrontMotor;
    private final DcMotor leftFrontMotor;
    private final DcMotor rightBackMotor;
    private final DcMotor leftBackMotor;

    private double leftBackMotorSpeed;
    private double rightBackMotorSpeed;
    private double rightFrontMotorSpeed;
    private double leftFrontMotorSpeed;

    public MecanumDrive(HardwareMap hardwareMap) {

        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void mecunamDrive(double y, double x, double rx) {

        leftFrontMotorSpeed = y + x + rx;
        leftBackMotorSpeed = y - x + rx;
        rightFrontMotorSpeed = y - x - rx;
        rightBackMotorSpeed = y + x - rx;

        double max = Math.max(Math.abs(leftFrontMotorSpeed), Math.abs(leftBackMotorSpeed));
        max = Math.max(max, Math.abs(rightFrontMotorSpeed));
        max = Math.max(max, Math.abs(rightBackMotorSpeed));

        if(max > 0.8) {
            leftFrontMotorSpeed  = (leftFrontMotorSpeed / max) * 0.8;
            rightFrontMotorSpeed   = (rightFrontMotorSpeed / max) * 0.8;
            leftBackMotorSpeed = (leftBackMotorSpeed / max) * 0.8;
            rightBackMotorSpeed  = (rightBackMotorSpeed / max) * 0.8;
        }

        leftFrontMotor.setPower(leftFrontMotorSpeed);
        rightFrontMotor.setPower(rightFrontMotorSpeed);
        leftBackMotor.setPower(leftBackMotorSpeed);
        rightBackMotor.setPower(rightBackMotorSpeed);
    }

    public double getLeftFrontPower() {
        return leftFrontMotorSpeed;
    }

    public double getRightFrontPower() {
        return rightFrontMotorSpeed;
    }

    public double getLeftBackPower() {
        return leftBackMotorSpeed;
    }

    public double getRightBackPower() {
        return rightBackMotorSpeed;
    }
}