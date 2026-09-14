package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private final DcMotor shooterMotor;

    private final ElapsedTime buttonTimer = new ElapsedTime();

    private boolean buttonWasPressed = false;

    private double shooterSpeed = 0;

    public Shooter(HardwareMap hardwareMap) {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
    }

    public void shooter(Gamepad gamepad) {

        // Button was just pressed
        if (gamepad.dpad_left && !buttonWasPressed) {

            buttonTimer.reset();
            buttonWasPressed = true;
        }

        // Button is being held
        if (gamepad.dpad_left) {

            double heldTime = buttonTimer.seconds();

            // Increase power based on how long the button is held
            shooterSpeed = heldTime * 0.2;

            // Maximum power is 1.0
            shooterSpeed = Math.min(shooterSpeed, 1.0);

            shooterMotor.setPower(shooterSpeed);
        }

        // Button was released
        if (!gamepad.dpad_left) {

            shooterSpeed = 0;
            shooterMotor.setPower(0);

            buttonWasPressed = false;
        }
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }
}