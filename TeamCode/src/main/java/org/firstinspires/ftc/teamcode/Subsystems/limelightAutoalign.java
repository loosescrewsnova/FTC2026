package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Computes turn/forward corrections so the robot faces the hive AprilTag
 * and sits at TARGET_DISTANCE from it. It does NOT touch the motors:
 * TeleOp reads turn/forward and applies them to the drive.
 */
public class limelightAutoalign {

    // ---------- Tunables (change while testing) ----------
    public static double kP_TURN = 0.015;       // negative if robot turns away from tag
    public static double kP_DIST = 0.02;        // negative if robot drives the wrong way
    public static double TARGET_DISTANCE = 60;  // FIND BY TESTING (same units as camera constants)
    public static double TURN_TOLERANCE = 1.0;  // degrees
    public static double DIST_TOLERANCE = 1.5;  // distance units
    public static double MAX_POWER = 0.4;       // cap on auto turn/forward power
    public static double MIN_POWER = 0.08;      // beats static friction near the target
    public static double READY_TIME = 0.2;      // seconds in tolerance before "ready"

    // ---------- Camera / field constants (MEASURE THESE) ----------
    public static double CAM_HEIGHT = 10;       // camera lens height off floor
    public static double TAG_HEIGHT = 30;       // center of tag off floor
    public static double CAM_ANGLE = 20;        // camera tilt, degrees up from horizontal

    private final Limelight3A limelight;
    private int tagId;
    private final ElapsedTime steadyTimer = new ElapsedTime();

    // ---------- Outputs (read these from TeleOp) ----------
    public double turn = 0;
    public double forward = 0;
    public double tx = 0;
    public double distance = 0;
    public boolean tagVisible = false;
    public boolean ready = false;

    public limelightAutoalign(Limelight3A limelight, int tagId) {
        this.limelight = limelight;
        this.tagId = tagId;
    }

    public void setTagId(int tagId) {
        this.tagId = tagId;
    }

    /** Call once per loop while the align button is held. */
    public void update() {
        turn = 0;
        forward = 0;
        tagVisible = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == tagId) {
                    tagVisible = true;
                    tx = tag.getTargetXDegrees();
                    double ty = tag.getTargetYDegrees();
                    distance = (TAG_HEIGHT - CAM_HEIGHT)
                            / Math.tan(Math.toRadians(CAM_ANGLE + ty));
                    break;
                }
            }
        }

        // Lost the tag: output nothing, never reuse old values
        if (!tagVisible) {
            ready = false;
            steadyTimer.reset();
            return;
        }

        double distError = distance - TARGET_DISTANCE;

        if (Math.abs(tx) > TURN_TOLERANCE) {
            turn = applyLimits(kP_TURN * tx);
        }
        if (Math.abs(distError) > DIST_TOLERANCE) {
            forward = applyLimits(kP_DIST * distError);
        }

        // Ready only after both errors stay inside tolerance for READY_TIME
        if (turn != 0 || forward != 0) {
            steadyTimer.reset();
        }
        ready = steadyTimer.seconds() > READY_TIME;
    }

    /** Call when the button is released so stale "ready" doesn't linger. */
    public void reset() {
        turn = 0;
        forward = 0;
        ready = false;
        steadyTimer.reset();
    }

    /** Clip to MAX_POWER, and bump up to MIN_POWER so small outputs still move the robot. */
    private double applyLimits(double power) {
        double clipped = Range.clip(power, -MAX_POWER, MAX_POWER);
        if (Math.abs(clipped) < MIN_POWER) {
            clipped = Math.signum(clipped) * MIN_POWER;
        }
        return clipped;
    }
}