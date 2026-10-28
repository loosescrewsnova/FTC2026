package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotor shooterMotor;
    private double currentPower = 0;

    private static final double SHOOTER_POWER = 0.7;

    public Shooter(HardwareMap hardwareMap) {

        // The shooter is currently configured as "intakeMotor".
        shooterMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);
    }

    public void shooter(Gamepad gamepad) {

        // Hold Y (top button) to shoot; release Y to stop.
        currentPower = gamepad.y ? SHOOTER_POWER : 0;
        shooterMotor.setPower(currentPower);
    }

    public void stop() {
        currentPower = 0;
        shooterMotor.setPower(0);
    }

    public double getShooterPower() {
        return currentPower;
    }
}