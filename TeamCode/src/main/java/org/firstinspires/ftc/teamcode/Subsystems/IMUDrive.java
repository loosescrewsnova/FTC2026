package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class IMUDrive {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private double leftPower;
    private double rightPower;

    // ----------------------------
    // ODOMETRY
    // ----------------------------

    private int previousLeft;
    private int previousRight;

    private double previousHeading = 0.0;

    private Pose2D robotPose = new Pose2D(
            DistanceUnit.INCH,
            0.0,
            0.0,
            AngleUnit.RADIANS,
            0.0
    );

    // CHANGE THESE TO MATCH YOUR MOTORS/WHEELS
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 3.7795;

    private static final double INCHES_PER_TICK =
            Math.PI * WHEEL_DIAMETER_INCHES / TICKS_PER_REV;

    public IMUDrive(HardwareMap hardwareMap) {

        leftMotor =
                hardwareMap.get(DcMotor.class, "leftMotor");

        rightMotor =
                hardwareMap.get(DcMotor.class, "rightMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // Reset encoders
        leftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        // Allows us to control the motors normally
        // while still reading their encoder positions.
        leftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        // Store starting encoder positions
        previousLeft = leftMotor.getCurrentPosition();
        previousRight = rightMotor.getCurrentPosition();
    }

    // ----------------------------
    // TANK DRIVE
    // ----------------------------

    public void IMUdrive(Gamepad gamepad) {

        leftPower = -gamepad.left_stick_y;
        rightPower = -gamepad.right_stick_y;

        // Limit maximum power to 0.8
        double max = Math.max(
                Math.abs(leftPower),
                Math.abs(rightPower)
        );

        if (max > 0.8) {
            leftPower = (leftPower / max) * 0.8;
            rightPower = (rightPower / max) * 0.8;
        }

        // Set motor power AFTER limiting it
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    // ----------------------------
    // ODOMETRY UPDATE
    // ----------------------------

    public void updateOdometry(IMU imu) {

        // Get current encoder positions
        int currentLeft = leftMotor.getCurrentPosition();
        int currentRight = rightMotor.getCurrentPosition();

        // Calculate encoder changes
        int deltaLeft = currentLeft - previousLeft;
        int deltaRight = currentRight - previousRight;

        // Save current positions for next update
        previousLeft = currentLeft;
        previousRight = currentRight;

        // Convert encoder ticks to inches
        double dLeft = deltaLeft * INCHES_PER_TICK;
        double dRight = deltaRight * INCHES_PER_TICK;

        /*
         * Average the two sides to determine
         * how far the robot traveled forward.
         */
        double robotForward =
                (dLeft + dRight) / 2.0;

        /*
         * Get heading from the IMU.
         */
        double heading =
                imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);

        /*
         * Calculate change in heading.
         */
        double deltaHeading =
                normalizeAngle(heading - previousHeading);

        /*
         * Use average heading during this update.
         */
        double averageHeading =
                previousHeading + deltaHeading / 2.0;

        previousHeading = heading;

        /*
         * Convert robot-relative forward movement
         * into field-relative X/Y movement.
         *
         * +X = right
         * +Y = forward
         */
        double fieldX =
                -robotForward * Math.sin(averageHeading);

        double fieldY =
                robotForward * Math.cos(averageHeading);

        /*
         * Update stored position.
         */
        double newX =
                robotPose.getX(DistanceUnit.INCH)
                        + fieldX;

        double newY =
                robotPose.getY(DistanceUnit.INCH)
                        + fieldY;

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                newX,
                newY,
                AngleUnit.RADIANS,
                heading
        );
    }

    // ----------------------------
    // ANGLE NORMALIZATION
    // ----------------------------

    private double normalizeAngle(double angle) {

        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    // ----------------------------
    // POSE
    // ----------------------------

    public Pose2D getPose() {
        return robotPose;
    }

    public double getX() {
        return robotPose.getX(DistanceUnit.INCH);
    }

    public double getY() {
        return robotPose.getY(DistanceUnit.INCH);
    }

    public double getHeadingRadians() {
        return robotPose.getHeading(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return robotPose.getHeading(AngleUnit.DEGREES);
    }

    // ----------------------------
    // RESET POSE
    // ----------------------------

    public void resetPose(IMU imu) {

        // Reset IMU yaw
        imu.resetYaw();

        // Reset stored pose
        robotPose = new Pose2D(
                DistanceUnit.INCH,
                0.0,
                0.0,
                AngleUnit.RADIANS,
                0.0
        );

        previousHeading = 0.0;

        // Reset encoder reference positions
        previousLeft =
                leftMotor.getCurrentPosition();

        previousRight =
                rightMotor.getCurrentPosition();
    }

    // ----------------------------
    // SET POSE
    // ----------------------------

    public void setPose(
            double x,
            double y,
            double headingRadians
    ) {

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.RADIANS,
                headingRadians
        );

        previousHeading = headingRadians;

        previousLeft =
                leftMotor.getCurrentPosition();

        previousRight =
                rightMotor.getCurrentPosition();
    }

    // ----------------------------
    // MOTOR POWER GETTERS
    // ----------------------------

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }
}