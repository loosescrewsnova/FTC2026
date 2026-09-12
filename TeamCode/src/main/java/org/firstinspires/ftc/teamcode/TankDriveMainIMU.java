package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Tank Drive Main IMU")
public class TankDriveMainIMU extends OpMode {

    // ----------------------------
    // MOTORS
    // ----------------------------

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private double leftPower;
    private double rightPower;

    // ----------------------------
    // IMU
    // ----------------------------

    private IMU imu;

    // ----------------------------
    // ENCODERS
    // ----------------------------

    private int previousLeft;
    private int previousRight;

    // ----------------------------
    // ODOMETRY
    // ----------------------------

    private double previousHeading = 0.0;

    private Pose2D robotPose = new Pose2D(
            DistanceUnit.INCH,
            0.0,
            0.0,
            AngleUnit.RADIANS,
            0.0
    );

    // Change these to match your robot
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 3.7795;

    private static final double INCHES_PER_TICK =
            Math.PI * WHEEL_DIAMETER_INCHES / TICKS_PER_REV;

    @Override
    public void init() {

        // ----------------------------
        // INITIALIZE MOTORS
        // ----------------------------

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

        // ----------------------------
        // RESET ENCODERS
        // ----------------------------

        leftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        leftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        // Store starting encoder positions
        previousLeft = leftMotor.getCurrentPosition();
        previousRight = rightMotor.getCurrentPosition();

        // ----------------------------
        // INITIALIZE IMU
        // ----------------------------

        imu = hardwareMap.get(IMU.class, "imu");

        // Reset IMU yaw
        imu.resetYaw();

        // ----------------------------
        // INITIALIZE POSE
        // ----------------------------

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                0.0,
                0.0,
                AngleUnit.RADIANS,
                0.0
        );

        previousHeading = 0.0;

        telemetry.addLine("Tank Drive IMU Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ----------------------------
        // TANK DRIVE
        // ----------------------------

        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        // Limit maximum power to 0.8
        double max = Math.max(
                Math.abs(leftPower),
                Math.abs(rightPower)
        );

        if (max > 0.8) {

            leftPower =
                    (leftPower / max) * 0.8;

            rightPower =
                    (rightPower / max) * 0.8;
        }

        // Set motor powers
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // ----------------------------
        // UPDATE ODOMETRY
        // ----------------------------

        updateOdometry();

        // ----------------------------
        // MOTOR TELEMETRY
        // ----------------------------

        telemetry.addData(
                "Left Power",
                "%.2f",
                leftPower
        );

        telemetry.addData(
                "Right Power",
                "%.2f",
                rightPower
        );

        // ----------------------------
        // ENCODER TELEMETRY
        // ----------------------------

        telemetry.addData(
                "Left Encoder",
                leftMotor.getCurrentPosition()
        );

        telemetry.addData(
                "Right Encoder",
                rightMotor.getCurrentPosition()
        );

        // ----------------------------
        // ODOMETRY TELEMETRY
        // ----------------------------

        telemetry.addData(
                "X Position",
                "%.2f in",
                getX()
        );

        telemetry.addData(
                "Y Position",
                "%.2f in",
                getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f degrees",
                getHeadingDegrees()
        );

        telemetry.update();
    }

    // ----------------------------
    // ODOMETRY
    // ----------------------------

    private void updateOdometry() {

        // Get current encoder positions
        int currentLeft =
                leftMotor.getCurrentPosition();

        int currentRight =
                rightMotor.getCurrentPosition();

        // Calculate encoder changes
        int deltaLeft =
                currentLeft - previousLeft;

        int deltaRight =
                currentRight - previousRight;

        // Save encoder positions
        previousLeft = currentLeft;
        previousRight = currentRight;

        // Convert encoder ticks to inches
        double dLeft =
                deltaLeft * INCHES_PER_TICK;

        double dRight =
                deltaRight * INCHES_PER_TICK;

        // Average distance traveled
        double robotForward =
                (dLeft + dRight) / 2.0;

        // Get IMU heading
        double heading =
                imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);

        // Calculate heading change
        double deltaHeading =
                normalizeAngle(
                        heading - previousHeading
                );

        // Average heading during this update
        double averageHeading =
                previousHeading
                        + deltaHeading / 2.0;

        previousHeading = heading;

        // Convert robot movement to field movement
        double fieldX =
                -robotForward
                        * Math.sin(averageHeading);

        double fieldY =
                robotForward
                        * Math.cos(averageHeading);

        // Update X/Y
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
    // NORMALIZE ANGLE
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
    // POSE GETTERS
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
        return robotPose.getHeading(
                AngleUnit.RADIANS
        );
    }

    public double getHeadingDegrees() {
        return robotPose.getHeading(
                AngleUnit.DEGREES
        );
    }
}
