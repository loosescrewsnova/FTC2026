package org.firstinspires.ftc.teamcode.Subsystems.servo;

/**
 * Tunable and hardware-mapping constants for the {@link LSServo} subsystem.
 *
 * <p>Positions are in the standard {@code Servo.setPosition()} range of 0.0-1.0,
 * where 0.0 and 1.0 map to the physical endpoints of the servo's travel
 * (e.g. 0 and 180 degrees on a standard REV Smart Robot Servo). Adjust
 * {@link #OPEN_POSITION} and {@link #CLOSED_POSITION} to match how the servo
 * is mounted on the claw.
 */
public final class ClawServoConstants {

    private ClawServoConstants() {
        // Utility class; no instances.
    }

    /** Hardware map device name for the claw servo. */
    public static final String CLAW_SERVO_NAME = "clawServo";

    /** Position the claw servo moves to when toggled "open". */
    public static final double OPEN_POSITION = 1.0;

    /** Position the claw servo moves to when toggled "closed" (also its initial position on construction). */
    public static final double CLOSED_POSITION = 0.0;
}

