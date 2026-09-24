package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LimelightLocalizer {

    private final Limelight3A limelight;

    private LLResult latestResult;

    public LimelightLocalizer(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Pipeline index for your AprilTag pipeline. Verify against
        // whatever you set up in the Limelight web UI.
        limelight.pipelineSwitch(8);
    }

    public void start() {
        limelight.start();
    }

    /*
     * Call once per loop, BEFORE reading pose. Limelight needs the current
     * robot yaw to disambiguate the AprilTag solve (MegaTag2), so pass in
     * the corrected heading from IMUOrthogonalNew.
     */
    public void update(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees);
        latestResult = limelight.getLatestResult();
    }

    public boolean hasValidTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /*
     * Botpose position comes back in meters by default - convert to inches
     * to match the units MecanumDrive's odometry uses everywhere else.
     * Verify getPosition()/toUnit() against your SDK version if this
     * doesn't compile - Limelight's FTC library has changed these names
     * across releases.
     */
    public double getFieldXInches() {

        Pose3D botPose = latestResult.getBotpose();
        Position position = botPose.getPosition().toUnit(DistanceUnit.INCH);

        return position.x;
    }

    public double getFieldYInches() {

        Pose3D botPose = latestResult.getBotpose();
        Position position = botPose.getPosition().toUnit(DistanceUnit.INCH);

        return position.y;
    }

    public double getFieldHeadingRadians() {

        Pose3D botPose = latestResult.getBotpose();

        return botPose.getOrientation().getYaw(AngleUnit.RADIANS);
    }

    public LLResult getLatestResult() {
        return latestResult;
    }
}