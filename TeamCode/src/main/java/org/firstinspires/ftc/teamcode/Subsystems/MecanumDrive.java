package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {

    private final DcMotor rightFrontMotor;
    private final DcMotor leftFrontMotor;
    private final DcMotor rightBackMotor;
    private final DcMotor leftBackMotor;

    private double leftBackMotorSpeed;
    private double rightBackMotorSpeed;
    private double rightFrontMotorSpeed;
    private double leftFrontMotorSpeed;

    // ----------------------------
    // ODOMETRY
    // ----------------------------

    private int previousLF;
    private int previousRF;
    private int previousLB;
    private int previousRB;

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

    // Calibrate experimentally. Start at 1.0.
    private static final double STRAFE_MULTIPLIER = 1.0;

    public MecanumDrive(HardwareMap hardwareMap) {

        rightFrontMotor =
                hardwareMap.get(DcMotor.class, "rightFrontMotor");

        leftFrontMotor =
                hardwareMap.get(DcMotor.class, "leftFrontMotor");

        rightBackMotor =
                hardwareMap.get(DcMotor.class, "rightBackMotor");

        leftBackMotor =
                hardwareMap.get(DcMotor.class, "leftBackMotor");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset drive encoders.
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_WITHOUT_ENCODER still allows encoder position reads.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        previousLF = leftFrontMotor.getCurrentPosition();
        previousRF = rightFrontMotor.getCurrentPosition();
        previousLB = leftBackMotor.getCurrentPosition();
        previousRB = rightBackMotor.getCurrentPosition();
    }

    public void mecanumDrive(double y, double x, double rx) {

        leftFrontMotorSpeed = y + x + rx;
        leftBackMotorSpeed = y - x + rx;
        rightFrontMotorSpeed = y - x - rx;
        rightBackMotorSpeed = y + x - rx;

        double max = Math.max(
                Math.abs(leftFrontMotorSpeed),
                Math.abs(leftBackMotorSpeed)
        );

        max = Math.max(max, Math.abs(rightFrontMotorSpeed));
        max = Math.max(max, Math.abs(rightBackMotorSpeed));

        if (max > 0.8) {
            leftFrontMotorSpeed = (leftFrontMotorSpeed / max) * 0.8;
            rightFrontMotorSpeed = (rightFrontMotorSpeed / max) * 0.8;
            leftBackMotorSpeed = (leftBackMotorSpeed / max) * 0.8;
            rightBackMotorSpeed = (rightBackMotorSpeed / max) * 0.8;
        }

        leftFrontMotor.setPower(leftFrontMotorSpeed);
        rightFrontMotor.setPower(rightFrontMotorSpeed);
        leftBackMotor.setPower(leftBackMotorSpeed);
        rightBackMotor.setPower(rightBackMotorSpeed);
    }

    // ----------------------------
    // ODOMETRY UPDATE
    // ----------------------------

    public void updateOdometry(IMU imu) {

        int currentLF = leftFrontMotor.getCurrentPosition();
        int currentRF = rightFrontMotor.getCurrentPosition();
        int currentLB = leftBackMotor.getCurrentPosition();
        int currentRB = rightBackMotor.getCurrentPosition();

        int deltaLF = currentLF - previousLF;
        int deltaRF = currentRF - previousRF;
        int deltaLB = currentLB - previousLB;
        int deltaRB = currentRB - previousRB;

        previousLF = currentLF;
        previousRF = currentRF;
        previousLB = currentLB;
        previousRB = currentRB;

        double dLF = deltaLF * INCHES_PER_TICK;
        double dRF = deltaRF * INCHES_PER_TICK;
        double dLB = deltaLB * INCHES_PER_TICK;
        double dRB = deltaRB * INCHES_PER_TICK;

        /*
         * Robot-relative movement.
         *
         * +forward = robot moves forward
         * +strafe  = robot moves right
         *
         * NOTE:
         * Encoder signs depend on your motor configuration.
         * Test by driving straight forward and verify all four
         * encoder deltas have the expected sign.
         */
        double robotForward =
                (dLF + dRF + dLB + dRB) / 4.0;

        double robotStrafe =
                ((dLF - dRF - dLB + dRB) / 4.0)
                        * STRAFE_MULTIPLIER;

        // IMU heading is the primary heading source.
        double heading =
                imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);

        /*
         * Use the average heading during the loop interval.
         * This improves the field-frame translation estimate
         * when rotating and translating at the same time.
         */
        double deltaHeading =
                normalizeAngle(heading - previousHeading);

        double averageHeading =
                previousHeading + deltaHeading / 2.0;

        previousHeading = heading;

        /*
         * Convert robot-relative translation into field-relative translation.
         *
         * Coordinate convention:
         * +X = right
         * +Y = forward
         * +heading = counterclockwise
         */
        double fieldX =
                robotStrafe * Math.cos(averageHeading)
                        - robotForward * Math.sin(averageHeading);

        double fieldY =
                robotStrafe * Math.sin(averageHeading)
                        + robotForward * Math.cos(averageHeading);

        double newX =
                robotPose.getX(DistanceUnit.INCH) + fieldX;

        double newY =
                robotPose.getY(DistanceUnit.INCH) + fieldY;

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                newX,
                newY,
                AngleUnit.RADIANS,
                heading
        );
    }

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

    public void resetPose(IMU imu) {

        imu.resetYaw();

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                0.0,
                0.0,
                AngleUnit.RADIANS,
                0.0
        );

        previousHeading = 0.0;

        previousLF = leftFrontMotor.getCurrentPosition();
        previousRF = rightFrontMotor.getCurrentPosition();
        previousLB = leftBackMotor.getCurrentPosition();
        previousRB = rightBackMotor.getCurrentPosition();
    }

    /*
     * Sets the stored XY pose.
     *
     * IMPORTANT:
     * The next updateOdometry() call will use the IMU heading as the
     * heading source. If you want an arbitrary field heading that differs
     * from the raw IMU yaw, add a heading offset to your IMU subsystem.
     */
    public void setPose(double x, double y, double headingRadians) {

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.RADIANS,
                headingRadians
        );

        previousHeading = headingRadians;

        previousLF = leftFrontMotor.getCurrentPosition();
        previousRF = rightFrontMotor.getCurrentPosition();
        previousLB = leftBackMotor.getCurrentPosition();
        previousRB = rightBackMotor.getCurrentPosition();
    }

    // ----------------------------
    // MOTOR POWER GETTERS
    // ----------------------------

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