package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NoOdometryMecanum {

    // These names must match your Driver Station configuration.
    private static final String FRONT_LEFT_NAME = "leftFrontMotor";
    private static final String FRONT_RIGHT_NAME = "rightFrontMotor";
    private static final String BACK_LEFT_NAME = "leftBackMotor";
    private static final String BACK_RIGHT_NAME = "rightBackMotor";

    // Ignore tiny joystick movements.
    private static final double DEADZONE = 0.05;

    // Maximum drivetrain power.
    private static final double MAX_POWER = 1.0;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public NoOdometryMecanum(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // Left side reversed so positive power = forward on every wheel.
        // Verify on the actual robot (see test steps below).
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{
                frontLeft, frontRight, backLeft, backRight
        }) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    // Gamepad 1:
    // Left stick Y  = forward / backward
    // Left stick X  = strafe left / right
    // Right stick X = rotate left / right
    public void driveWithGamepad(Gamepad gamepad) {

        double forward = applyDeadzone(-gamepad.left_stick_y); // stick up is negative
        double strafe  = applyDeadzone(gamepad.left_stick_x);  // right is positive
        double turn    = applyDeadzone(gamepad.right_stick_x); // right = clockwise

        setDrivePowers(forward, strafe, turn);
    }

    // forward: + = forward, strafe: + = right, turn: + = clockwise
    public void setDrivePowers(double forward, double strafe, double turn) {

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Scale all powers down together if any exceeds 1.0, preserving direction.
        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        frontLeft.setPower((fl / max) * MAX_POWER);
        frontRight.setPower((fr / max) * MAX_POWER);
        backLeft.setPower((bl / max) * MAX_POWER);
        backRight.setPower((br / max) * MAX_POWER);
    }

    public void stop() {
        setDrivePowers(0, 0, 0);
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }
}