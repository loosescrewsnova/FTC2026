package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -1.0;

    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    private boolean previousB = false;
    private boolean previousA = false;

    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);
    }

    public void intake(Gamepad gamepad) {

        boolean bPressed = gamepad.b;
        boolean aPressed = gamepad.a;


        if (bPressed && !previousB) {
            intakeOn = !intakeOn;
            outtakeOn = false;
        }

        if (aPressed && !previousA) {
            outtakeOn = !outtakeOn;
            intakeOn = false;
        }

        previousB = bPressed;
        previousA = aPressed;

        if (outtakeOn) {
            intakeMotor.setPower(OUTTAKE_POWER);
        } else if (intakeOn) {
            intakeMotor.setPower(INTAKE_POWER);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void stop() {
        intakeOn = false;
        outtakeOn = false;
        intakeMotor.setPower(0);
    }

    public static double getIntakePower() {
        return INTAKE_POWER;
    }

    public static double getOuttakePower() {
        return OUTTAKE_POWER;
    }

    public void run() {
    }
}