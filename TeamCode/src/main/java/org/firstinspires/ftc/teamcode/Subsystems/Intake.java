package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -1.0;
    private static final double HOLD_TIME = 1.5;

    private boolean intakeOn = false;

    private boolean previousB = false;
    private boolean outtakeActive = false;

    private long bPressTime = 0;

    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);
    }

    public void intake(Gamepad gamepad) {

        boolean bPressed = gamepad.b;

        // Detect initial B press.
        if (bPressed && !previousB) {
            bPressTime = System.nanoTime();
            outtakeActive = false;
        }

        // Check if B is held for 1.5 seconds.
        if (bPressed && !outtakeActive) {

            double holdTime =
                    (System.nanoTime() - bPressTime) / 1_000_000_000.0;

            if (holdTime >= HOLD_TIME) {
                outtakeActive = true;
            }
        }

        // Detect B release.
        if (!bPressed && previousB) {

            if (!outtakeActive) {
                // Short tap toggles intake.
                intakeOn = !intakeOn;
            }

            outtakeActive = false;
        }

        previousB = bPressed;

        // Control motor.
        if (outtakeActive) {
            intakeMotor.setPower(OUTTAKE_POWER);
        } else if (intakeOn) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void stop() {
        intakeOn = false;
        outtakeActive = false;
        intakeMotor.setPower(0);
    }

    public static double getIntakePower() {
        return INTAKE_POWER;
    }

    public static double getOuttakePower() {
        return OUTTAKE_POWER;
    }
}