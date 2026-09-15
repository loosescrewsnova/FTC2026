package org.firstinspires.ftc.teamcode.Subsystems.shooter;

/**
 * Tunable and hardware-mapping constants for the {@link Shooter} subsystem.
 */
public final class ShooterConstants {

    private ShooterConstants() {
        // Utility class; no instances.
    }

    /** Hardware map device name for the shooter motor. */
    public static final String SHOOTER_MOTOR_NAME = "shooterMotor";

    /** Rate at which shooter power ramps up per second the trigger button is held. */
    public static final double POWER_RAMP_RATE = 0.2;

    /** Maximum allowed shooter motor power. */
    public static final double MAX_SHOOTER_POWER = 1.0;

    /** Idle/reset power for the shooter motor (also its initial power on construction). */
    public static final double IDLE_POWER = 0.0;
}
