package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;
    private double intakePower = 0;

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double INTAKE_POWER = 1.0;

    public Intake(HardwareMap hardwareMap) {

        // The robot configuration has the shooter and intake names swapped,
        // so the intake is the motor configured as "shooterMotor".
        intakeMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        // Reverse the motor if needed for the intake mechanism
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    public void intake(Gamepad gamepad) {

        // Hold the RIGHT trigger to run the intake
        if (gamepad.right_trigger > TRIGGER_THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else {
            intakePower = 0;
        }

        intakeMotor.setPower(intakePower);
    }

    public void stop() {
        intakePower = 0;
        intakeMotor.setPower(0);
    }

    public double getIntakePower() {
        return intakePower;
    }
}