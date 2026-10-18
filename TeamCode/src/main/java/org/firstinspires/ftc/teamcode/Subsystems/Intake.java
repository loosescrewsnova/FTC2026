
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

        intakeMotor = hardwareMap.get(
                DcMotor.class,
                "intakeMotor"
        );

        // Reverse the motor if needed for the intake mechanism
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        intakeMotor.setPower(0);
    }

    public void intake(Gamepad gamepad) {

        // Hold the left trigger to run the intake
        if (gamepad.left_trigger > TRIGGER_THRESHOLD) {
            intakePower = INTAKE_POWER;
        } else {
            intakePower = 0;
        }

        intakeMotor.setPower(intakePower);
    }

    public double getIntakePower() {
        return intakePower;
    }
}
