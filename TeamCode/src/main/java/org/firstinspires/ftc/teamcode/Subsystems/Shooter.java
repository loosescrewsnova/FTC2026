
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotor shooterMotor;
    private double currentPower = 0;

    private boolean isY = false;

    private static final double SHOOTER_POWER = 0.7;

    public Shooter(HardwareMap hardwareMap) {

        // The shooter is currently configured as "intakeMotor".
        shooterMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Reverse the motor if needed for the correct shooting direction.
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);
    }

    public void shooter(Gamepad gamepad) {

        shooterMotor.setPower(0);

        while(gamepad.y) {
            shooterMotor.setPower(1);

            // Time since the last loop, so the ramp is based on real time
            double now = timer.seconds();
            double dt = Math.min(now - lastTime, MAX_DT);
            lastTime = now;

            // Hold d-pad right to speed up, d-pad left to slow down
            if (gamepad.dpad_right) {
                shooterSpeed += RAMP_RATE * dt;

                if(shooterSpeed < 0) {
                    shooterSpeed = 0;
                }
            }
            if (gamepad.dpad_left) {
                shooterSpeed -= RAMP_RATE * dt;

                if(shooterSpeed < 0) {
                    shooterSpeed = 0;
                }
            }

            // Keep it inside the allowed range
            shooterSpeed = Math.max(MIN_SPEED, Math.min(MAX_SPEED, shooterSpeed));

            shooterMotor.setPower(shooterSpeed);
        }
    }

    public void stop() {
        shooterMotor.setPower(0);
    }

    public Object getShooterSpeed() {
        return null;
    }
}
