package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    // ODOMETRY (two dead-wheel pods + IMU heading)
    // ----------------------------

    // The pods are read through whatever motor port their encoder cable
    // is plugged into. These are the hardware-config names of those ports.
    // If a pod shares a port with a drive motor, use that drive motor's
    // config name here (and note that reversing that motor in code also
    // flips its encoder count - just flip the DIRECTION constant below).
    private static final String FORWARD_POD_NAME = "forwardPod";
    private static final String STRAFE_POD_NAME = "strafePod";

    private final DcMotor forwardPod;
    private final DcMotor strafePod;

    private int previousForwardTicks;
    private int previousStrafeTicks;

    private double previousHeading = 0.0;
    private boolean headingInitialized = false;

    private Pose2D robotPose = new Pose2D(
            DistanceUnit.INCH,
            0.0,
            0.0,
            AngleUnit.RADIANS,
            0.0
    );

    // CHANGE THESE TO MATCH YOUR PODS
    // (defaults are the goBILDA 4-Bar pod: 2000 ticks/rev, 32 mm wheel)
    private static final double POD_TICKS_PER_REV = 2000.0;
    private static final double POD_WHEEL_DIAMETER_INCHES = 1.2598;

    private static final double INCHES_PER_TICK =
            Math.PI * POD_WHEEL_DIAMETER_INCHES / POD_TICKS_PER_REV;

    // Flip to -1 if a pod counts the wrong way.
    // Forward pod must count UP when the robot is pushed forward.
    // Strafe pod must count UP when the robot is pushed to the RIGHT.
    private static final int FORWARD_POD_DIRECTION = 1;
    private static final int STRAFE_POD_DIRECTION = 1;

    // Calibrate experimentally. Start at 1.0.
    // multiplier = (real distance pushed) / (distance odometry reported)
    private static final double FORWARD_MULTIPLIER = 1.0;
    private static final double STRAFE_MULTIPLIER = 1.0;

    // MEASURE THESE (inches, from the robot's center of rotation).
    //
    // FORWARD_POD_Y_INCHES: how far the forward (parallel) pod is to the
    //   LEFT of center. Left is positive, right is negative.
    // STRAFE_POD_X_INCHES: how far the strafe (perpendicular) pod is
    //   FORWARD of center. Forward is positive, behind is negative.
    //
    // These cancel out the distance the pods "see" when the robot only
    // turns. If a pod sits exactly on the center line, its offset is 0.
    private static final double FORWARD_POD_Y_INCHES = 0.0;
    private static final double STRAFE_POD_X_INCHES = 0.0;

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

        // Drive motors no longer feed odometry, so they just run open loop.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry pods.
        forwardPod = hardwareMap.get(DcMotor.class, FORWARD_POD_NAME);
        strafePod = hardwareMap.get(DcMotor.class, STRAFE_POD_NAME);

        forwardPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafePod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // RUN_WITHOUT_ENCODER still allows encoder position reads.
        forwardPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafePod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        previousForwardTicks = forwardPod.getCurrentPosition();
        previousStrafeTicks = strafePod.getCurrentPosition();
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

    /*
     * headingRadians should come from IMUOrthogonalNew.getYawRadians(),
     * NOT read directly from the raw IMU. That way any correction
     * Limelight has applied to the IMU (via correctYaw()) automatically
     * flows into the position calculation below.
     *
     * Field frame (unchanged from the encoder version):
     *   heading 0 -> robot forward = field +Y, robot right = field +X
     *   heading is counterclockwise-positive
     */
    public void updateOdometry(double headingRadians) {

        // First call: latch the current heading so we don't treat
        // "IMU isn't at zero yet" as a giant rotation.
        if (!headingInitialized) {
            previousHeading = headingRadians;
            headingInitialized = true;
        }

        int currentForwardTicks = forwardPod.getCurrentPosition();
        int currentStrafeTicks = strafePod.getCurrentPosition();

        int deltaForwardTicks = currentForwardTicks - previousForwardTicks;
        int deltaStrafeTicks = currentStrafeTicks - previousStrafeTicks;

        previousForwardTicks = currentForwardTicks;
        previousStrafeTicks = currentStrafeTicks;

        // What each pod physically rolled, in inches.
        double measuredForward =
                deltaForwardTicks * FORWARD_POD_DIRECTION
                        * INCHES_PER_TICK * FORWARD_MULTIPLIER;

        double measuredStrafe =
                deltaStrafeTicks * STRAFE_POD_DIRECTION
                        * INCHES_PER_TICK * STRAFE_MULTIPLIER;

        double deltaHeading =
                normalizeAngle(headingRadians - previousHeading);

        // Remove the part of each pod's reading caused purely by the robot
        // rotating (pods that are off-center travel an arc when you turn).
        double robotForward =
                measuredForward + deltaHeading * FORWARD_POD_Y_INCHES;

        double robotStrafe =
                measuredStrafe + deltaHeading * STRAFE_POD_X_INCHES;

        double averageHeading =
                previousHeading + deltaHeading / 2.0;

        previousHeading = headingRadians;

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
                headingRadians
        );
    }

    // ----------------------------
    // VISION POSITION CORRECTION (Limelight -> Odometry)
    // ----------------------------

    /*
     * Call whenever Limelight has a valid AprilTag detection, using the
     * field X/Y it computed from the tag. Heading is left untouched here
     * since that correction already happened in IMUOrthogonalNew.
     */
    public void applyVisionCorrection(double xInches, double yInches) {

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                xInches,
                yInches,
                AngleUnit.RADIANS,
                robotPose.getHeading(AngleUnit.RADIANS)
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

    public void resetPose() {

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                0.0,
                0.0,
                AngleUnit.RADIANS,
                0.0
        );

        // Re-latch to whatever heading the IMU reports next.
        headingInitialized = false;

        previousForwardTicks = forwardPod.getCurrentPosition();
        previousStrafeTicks = strafePod.getCurrentPosition();
    }

    public void setPose(double x, double y, double headingRadians) {

        robotPose = new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.RADIANS,
                headingRadians
        );

        previousHeading = headingRadians;
        headingInitialized = true;

        previousForwardTicks = forwardPod.getCurrentPosition();
        previousStrafeTicks = strafePod.getCurrentPosition();
    }

    // ----------------------------
    // RAW POD TICKS (for checking directions / calibrating)
    // ----------------------------

    public int getForwardPodTicks() {
        return forwardPod.getCurrentPosition() * FORWARD_POD_DIRECTION;
    }

    public int getStrafePodTicks() {
        return strafePod.getCurrentPosition() * STRAFE_POD_DIRECTION;
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