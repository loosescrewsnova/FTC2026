package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor;
    private double intakePower;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakePower = 1;
    }

    public void intake(Gamepad gamepad) {

        if (gamepad.b) {
            intakePower = -1;
        } else if (gamepad.a) {
            intakePower = 1;
        } else {
            intakePower = 0;
        }

        intakeMotor.setPower(intakePower);
    }

    public double getIntakePower() {
        return intakePower;
    }
}