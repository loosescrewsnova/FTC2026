
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotor shooterMotor;

    private static final double SHOOTER_POWER = 1.0;

    public Shooter(HardwareMap hardwareMap) {

        // The shooter is currently configured as "intakeMotor".
        shooterMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Reverse the motor if needed for the correct shooting direction.
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0);
    }

    public void shooter(Gamepad gamepad) {

        // Hold Y to shoot; release Y to stop.
        if (gamepad.y) {
            shooterMotor.setPower(SHOOTER_POWER);
        } else {
            shooterMotor.setPower(0);
        }
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
}
