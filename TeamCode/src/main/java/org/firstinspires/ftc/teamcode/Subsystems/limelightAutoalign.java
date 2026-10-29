package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.HashSet;
import java.util.Set;

/**
 * Turns the robot to face the hive AprilTag and drives it to exactly
 * TARGET_DISTANCE (5.2 ft = 62.4 in) from the tag.
 *
 * It does NOT touch the motors. TeleOp adds turn/forward into the mecanum
 * drive, which sends power to all four wheels.
 *
 * ALL DISTANCES ARE IN INCHES.
 */
public class limelightAutoalign {

    // ---------- Target ----------
    public static double TARGET_DISTANCE = 62.4;   // 5.2 ft in inches

    // ---------- Tunables ----------
    public static double kP_TURN = 0.015;
    public static double kP_DIST = 0.02;
    public static double TURN_TOLERANCE = 1.0;     // degrees
    public static double DIST_TOLERANCE = 1.0;     // inches (0.5 made it chatter in simulation)
    public static double MAX_POWER = 0.4;
    public static double MIN_POWER = 0.08;
    public static double READY_TIME = 0.2;         // seconds
    public static double FILTER_ALPHA = 0.5;       // 1 = no smoothing, lower = smoother
    public static double MAX_STALENESS_MS = 100;   // ignore camera data older than this

    // ---------- Camera constants (MEASURE THESE, in inches / degrees) ----------
    public static double CAM_HEIGHT = 10;          // lens height off floor
    public static double TAG_HEIGHT = 30;          // center of tag off floor
    public static double CAM_ANGLE = 20;           // camera tilt up from horizontal

    private final Limelight3A limelight;
    private final Set<Integer> allowedTags = new HashSet<>();
    private final ElapsedTime steadyTimer = new ElapsedTime();
    private boolean haveFilteredDistance = false;

    // ---------- Outputs ----------
    public double turn = 0;          // + = rotate clockwise (same as rx in your drive)
    public double forward = 0;       // + = drive robot forward
    public double tx = 0;
    public double distance = 0;      // filtered distance, inches
    public double distanceError = 0; // distance - TARGET_DISTANCE
    public int lockedTagId = -1;
    public boolean tagVisible = false;
    public boolean ready = false;

    public limelightAutoalign(Limelight3A limelight, int... tagIds) {
        this.limelight = limelight;
        setTagIds(tagIds);
    }

    public void setTagId(int tagId) {
        setTagIds(tagId);
    }

    public void setTagIds(int... tagIds) {
        allowedTags.clear();
        for (int id : tagIds) allowedTags.add(id);
    }

    /** Call once per loop while the align button is held. */
    public void update() {
        turn = 0;
        forward = 0;
        tagVisible = false;
        lockedTagId = -1;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < MAX_STALENESS_MS) {
            LLResultTypes.FiducialResult best = null;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (!allowedTags.contains(tag.getFiducialId())) continue;
                if (best == null
                        || Math.abs(tag.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees())) {
                    best = tag;
                }
            }
            // Total angle above horizontal must be positive or the distance math breaks
            double angle = (best == null) ? 0 : CAM_ANGLE + best.getTargetYDegrees();
            if (best != null && angle > 1.0) {
                tagVisible = true;
                lockedTagId = best.getFiducialId();
                tx = best.getTargetXDegrees();
                double rawDistance = (TAG_HEIGHT - CAM_HEIGHT)
                        / Math.tan(Math.toRadians(angle));

                // Low-pass filter so camera noise doesn't make the robot twitch
                if (!haveFilteredDistance) {
                    distance = rawDistance;
                    haveFilteredDistance = true;
                } else {
                    distance = FILTER_ALPHA * rawDistance + (1 - FILTER_ALPHA) * distance;
                }
            }
        }

        // Lost the tag: output nothing, never reuse old values
        if (!tagVisible) {
            haveFilteredDistance = false;
            ready = false;
            steadyTimer.reset();
            return;
        }

        distanceError = distance - TARGET_DISTANCE;

        if (Math.abs(tx) > TURN_TOLERANCE) {
            turn = applyLimits(kP_TURN * tx);
        }

        if (Math.abs(distanceError) > DIST_TOLERANCE) {
            // Too far (error > 0) -> drive toward the tag. Too close -> back away.
            // If the robot drives the wrong way, make kP_DIST negative.
            forward = applyLimits(kP_DIST * distanceError);
        }

        // Ready only after both errors stay inside tolerance for READY_TIME
        if (turn != 0 || forward != 0) {
            steadyTimer.reset();
        }
        ready = steadyTimer.seconds() > READY_TIME;
    }

    /** Call when the button is released. */
    public void reset() {
        turn = 0;
        forward = 0;
        ready = false;
        haveFilteredDistance = false;
        steadyTimer.reset();
    }

    private double applyLimits(double power) {
        double clipped = Range.clip(power, -MAX_POWER, MAX_POWER);
        if (Math.abs(clipped) < MIN_POWER) {
            clipped = Math.signum(clipped) * MIN_POWER;
        }
        return clipped;
    }
}