package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Testing")
public class IntakeTesting extends OpMode {

    private DcMotor intakeMotor;
    private double intakePower;

    boolean wasA, isA, isB, wasB;

    @Override
    public void init() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakePower = 1;
    }

    @Override
    public void loop() {

        isA = gamepad1.a;
        isB = gamepad1.b;
        if (isB && !wasB) {
            intakePower = -1;
        } else if (isA && !wasA) {
            intakePower = 1;
        }
        wasA = isA;
        wasB = isB;
        intakeMotor.setPower(intakePower);
    }
}