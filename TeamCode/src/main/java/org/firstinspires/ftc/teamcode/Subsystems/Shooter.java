
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotor shooterMotor;

    private double currentPower = 0;
    private boolean shooterOn = false;
    private boolean previousY = false;

    private static final double SHOOTER_POWER = 0.622;

    public Shooter(HardwareMap hardwareMap) {

        // The shooter is currently configured as "intakeMotor".
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);
    }

    public void shooter(Gamepad gamepad) {

        // Detect a new Y button press.
        if (gamepad.y && !previousY) {
            shooterOn = !shooterOn;
        }

        previousY = gamepad.y;

        // Set shooter power.
        currentPower = shooterOn ? SHOOTER_POWER : 0;

        shooterMotor.setPower(currentPower);
    }

    public void stop() {
        shooterOn = false;
        currentPower = 0;
        shooterMotor.setPower(0);
    }

    public double getShooterPower() {
        return currentPower;
    }
}
