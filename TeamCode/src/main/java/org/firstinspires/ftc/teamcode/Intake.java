package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor;
    private double intakePower;

    boolean wasA, isA, isB, wasB;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakePower = 1;
    }

    public void intake(Gamepad gamepad) {

        isA = gamepad.a;
        isB = gamepad.b;
        if (isB && !wasB) {
            intakePower = -1;
        } else if (isA && !wasA) {
            intakePower = 1;
        }
        wasA = isA;
        wasB = isB;
        intakeMotor.setPower(intakePower);
    }

    public double getIntakePower() {
        return intakePower;
    }
}