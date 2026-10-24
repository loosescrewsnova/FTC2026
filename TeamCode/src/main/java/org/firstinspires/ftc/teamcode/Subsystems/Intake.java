package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;
    private double intakePower = 0;

    private static final double INTAKE_POWER = 1.0;

    private boolean wasBPressed = false;

    public Intake(HardwareMap hardwareMap) {

        // The intake is currently configured as "shooterMotor".
        intakeMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    public void intake(Gamepad gamepad) {

        intakeMotor.setPower(0);

        while(gamepad.b) {
            intakeMotor.setPower(1);
        }
    }

    public void stop() {
        intakePower = 0;
        intakeMotor.setPower(0);
    }

    public void stop() {
        intakePower = 0;
        intakeMotor.setPower(0);
    }

    public void stop() {
        intakePower = 0;
        intakeMotor.setPower(0);
    }

    public double getIntakePower() {
        return intakePower;
    }
}
