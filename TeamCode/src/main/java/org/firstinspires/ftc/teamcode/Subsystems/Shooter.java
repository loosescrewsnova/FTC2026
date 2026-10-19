package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter controlled only by the d-pad (no bumper needed).
 *
 *   Hold d-pad right = speed goes up   (0.2 per second)
 *   Hold d-pad left  = speed goes down (0.2 per second)
 *
 * The motor always runs at the current speed. It starts at 0 (off), so
 * hold d-pad right to spin it up and d-pad left down to 0 to turn it off.
 */
public class Shooter {

    private final DcMotor shooterMotor;

    // Speed limits. Speed can never go below 0; 1.0 is the motor's max power.
    private static final double MIN_SPEED = 0.0;
    private static final double MAX_SPEED = 1.0;

    // Speed when the opmode starts (0 = off)
    private static final double STARTING_SPEED = 0.0;

    // How fast the speed changes while d-pad left/right is held: 0.2 per second
    private static final double RAMP_RATE = 0.2;

    // Safety cap on dt so a long pause between loops can't cause a big jump
    private static final double MAX_DT = 0.1;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime;

    private double shooterSpeed = STARTING_SPEED;

    public Shooter(HardwareMap hardwareMap) {
        // The robot configuration has the shooter and intake names swapped,
        // so the shooter is the motor configured as "intakeMotor".
        shooterMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Flip to REVERSE if the shooter spins the wrong way
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);

        timer.reset();
        lastTime = timer.seconds();
    }

    public void shooter(Gamepad gamepad) {

        // Time since the last loop, so the speed change is based on real time
        double now = timer.seconds();
        double dt = Math.min(now - lastTime, MAX_DT);
        lastTime = now;

        // d-pad right = faster, d-pad left = slower (pressing both does nothing)
        if (gamepad.dpad_right && !gamepad.dpad_left) {
            shooterSpeed += RAMP_RATE * dt;
        } else if (gamepad.dpad_left && !gamepad.dpad_right) {
            shooterSpeed -= RAMP_RATE * dt;
        }

        // Keep it inside the allowed range (never below 0)
        shooterSpeed = Math.max(MIN_SPEED, Math.min(MAX_SPEED, shooterSpeed));

        shooterMotor.setPower(shooterSpeed);
    }

    // Call right after waitForStart() so the first loop's dt isn't huge
    public void resetTimer() {
        timer.reset();
        lastTime = timer.seconds();
    }

    public void stop() {
        shooterSpeed = 0;
        shooterMotor.setPower(0);
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    // True whenever the shooter speed is above 0
    public boolean isRunning() {
        return shooterSpeed > 0;
    }
}