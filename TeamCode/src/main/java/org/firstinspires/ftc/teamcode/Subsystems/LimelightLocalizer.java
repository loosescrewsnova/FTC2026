package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class LimelightLocalizer {

    // Only these AprilTag IDs count for localization.
    private static final Set<Integer> ALLOWED_TAGS =
            new HashSet<>(Arrays.asList(34, 35, 36, 37));

    private final Limelight3A limelight;
    private LLResult latestResult;
    private boolean seesAllowedTag = false;

    public LimelightLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // your AprilTag pipeline
    }

    public void start() {
        limelight.start();
    }

    /** Call once per loop BEFORE reading pose. Pass the corrected robot yaw in degrees. */
    public void update(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees);
        latestResult = limelight.getLatestResult();

        seesAllowedTag = false;
        if (latestResult != null && latestResult.isValid()) {
            for (LLResultTypes.FiducialResult tag : latestResult.getFiducialResults()) {
                if (ALLOWED_TAGS.contains(tag.getFiducialId())) {
                    seesAllowedTag = true;
                    break;
                }
            }
        }
    }

    /** True only if the Limelight currently sees at least one of tags 34-37. */
    public boolean hasValidTarget() {
        return seesAllowedTag;
    }

    public double getFieldXInches() {
        Pose3D pose = latestResult.getBotpose();
        return pose.getPosition().toUnit(DistanceUnit.INCH).x;
    }

    public double getFieldYInches() {
        Pose3D pose = latestResult.getBotpose();
        return pose.getPosition().toUnit(DistanceUnit.INCH).y;
    }

    public double getFieldHeadingRadians() {
        Pose3D pose = latestResult.getBotpose();
        return pose.getOrientation().getYaw(AngleUnit.RADIANS);
    }

    public double getTx() {
        return 0;
    }
}