package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;
    private double intakePower = 0;

    // Trigger values range 0.0-1.0; anything above this counts as "held"
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double INTAKE_POWER = 1.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Flip to FORWARD if the intake spins the wrong way
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(Gamepad gamepad) {

        intakeMotor.setPower(0);

        while(gamepad.b) {
            intakeMotor.setPower(1);
        }
    }

    public double getIntakePower() {
        return intakePower;
    }
}