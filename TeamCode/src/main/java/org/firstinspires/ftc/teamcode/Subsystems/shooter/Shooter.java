package org.firstinspires.ftc.teamcode.Subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private final DcMotor shooterMotor;

    private final ElapsedTime buttonTimer = new ElapsedTime();

    private boolean buttonWasPressed = false;

    private double shooterSpeed = ShooterConstants.IDLE_POWER;

    public Shooter(HardwareMap hardwareMap) {

        shooterMotor = hardwareMap.get(DcMotor.class, ShooterConstants.SHOOTER_MOTOR_NAME);
    }

    public void shooter(Gamepad gamepad) {

        // Button was just pressed
        if (gamepad.dpad_left && !buttonWasPressed) {

            buttonTimer.reset();
            buttonWasPressed = true;
        }

        // Button is being held
        if (gamepad.dpad_left && shooterSpeed < ShooterConstants.MAX_SHOOTER_POWER) {

            double heldTime = buttonTimer.seconds();

            // Increase power based on how long the button is held
            shooterSpeed = heldTime * ShooterConstants.POWER_RAMP_RATE;

            // Cap power at the configured maximum
            //shooterSpeed = Math.min(shooterSpeed, ShooterConstants.MAX_SHOOTER_POWER); NOT required as the if condition does not run the unnecessary code

            shooterMotor.setPower(shooterSpeed);
        }

        // Button was released
        if (!gamepad.dpad_left) {

            shooterSpeed = ShooterConstants.IDLE_POWER;
            shooterMotor.setPower(ShooterConstants.IDLE_POWER);

            buttonWasPressed = false;
        }
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }
}
