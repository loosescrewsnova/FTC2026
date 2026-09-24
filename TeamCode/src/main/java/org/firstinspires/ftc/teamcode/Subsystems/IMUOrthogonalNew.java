package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUOrthogonalNew {

    private final IMU imu;

    private final RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    private final RevHubOrientationOnRobot.UsbFacingDirection usbDirection;

    // ----------------------------
    // VISION HEADING CORRECTION
    // ----------------------------

    /*
     * Raw IMU yaw drifts slowly over a match (gyro bias). Whenever
     * Limelight sees a valid AprilTag, correctYaw() is called with the
     * vision-derived heading, and this offset updates so getYawRadians()/
     * getYawDegrees() report a corrected heading without ever touching
     * the physical IMU itself.
     */
    private double yawOffsetRadians = 0.0;

    public IMUOrthogonalNew(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        // Define how the Control Hub is mounted
        logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public IMU getIMU() {
        return imu;
    }

    public void resetYaw() {
        imu.resetYaw();
        yawOffsetRadians = 0.0;
    }

    // ----------------------------
    // CORRECTED YAW (Limelight -> IMU)
    // ----------------------------

    /*
     * Call whenever Limelight has a valid AprilTag detection.
     * visionYawRadians should be the field-relative heading Limelight
     * computed from the tag (Pose3D botpose orientation).
     */
    public void correctYaw(double visionYawRadians) {
        yawOffsetRadians = normalizeAngle(visionYawRadians - getRawYawRadians());
    }

    private double getRawYawRadians() {

        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public double getYawRadians() {
        return normalizeAngle(getRawYawRadians() + yawOffsetRadians);
    }

    public double getYawDegrees() {
        return Math.toDegrees(getYawRadians());
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

    public double getPitchDegrees() {

        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        return orientation.getPitch(AngleUnit.DEGREES);
    }

    public double getPitchRadians() {

        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        return orientation.getPitch(AngleUnit.RADIANS);
    }

    public double getRollDegrees() {

        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        return orientation.getRoll(AngleUnit.DEGREES);
    }

    public double getRollRadians() {

        YawPitchRollAngles orientation =
                imu.getRobotYawPitchRollAngles();

        return orientation.getRoll(AngleUnit.RADIANS);
    }

    public double getYawVelocity() {

        AngularVelocity angularVelocity =
                imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        return angularVelocity.zRotationRate;
    }

    public double getPitchVelocity() {

        AngularVelocity angularVelocity =
                imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        return angularVelocity.xRotationRate;
    }

    public double getRollVelocity() {

        AngularVelocity angularVelocity =
                imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        return angularVelocity.yRotationRate;
    }

    public String getLogoDirection() {
        return logoDirection.toString();
    }

    public String getUsbDirection() {
        return usbDirection.toString();
    }
}