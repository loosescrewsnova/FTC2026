package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple robot-centric mecanum drive. No odometry, no localizer, no encoders.
 *
 * Controls (gamepad1):
 *   Left stick  = drive forward/back and strafe left/right
 *   Right stick X = turn
 *
 * The d-pad and bumpers/triggers are NOT used here, so they're free for the
 * shooter, intake and feeder.
 */
public class NoOdometryMecanum {

    // Change these to match the names in your Driver Station configuration
    private static final String FRONT_LEFT_NAME = "leftFrontMotor";
    private static final String FRONT_RIGHT_NAME = "rightFrontMotor";
    private static final String BACK_LEFT_NAME = "leftBackMotor";
    private static final String BACK_RIGHT_NAME = "rightBackMotor";

    // Ignore tiny stick movements so the robot doesn't creep
    private static final double DEADZONE = 0.05;

    // Lower this (e.g. 0.7) if the robot is too fast to control
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

        // Left side reversed is standard for most mecanum robots.
        // If the robot drives backwards or spins when you push forward,
        // swap FORWARD/REVERSE on the motors that are wrong.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    public void driveWithGamepad(Gamepad gamepad) {
        // Stick Y is negative when pushed forward, so flip it
        double forward = applyDeadzone(gamepad.left_stick_y);
        double strafe = applyDeadzone(-gamepad.left_stick_x);
        double turn = applyDeadzone(-gamepad.right_stick_x);

        setDrivePowers(forward, strafe, turn);
    }

    public void setDrivePowers(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Scale everything down if any motor would go past 1.0
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max * MAX_POWER);
        frontRight.setPower(fr / max * MAX_POWER);
        backLeft.setPower(bl / max * MAX_POWER);
        backRight.setPower(br / max * MAX_POWER);
    }

    public void stop() {
        setDrivePowers(0, 0, 0);
    }

    private static double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }
}