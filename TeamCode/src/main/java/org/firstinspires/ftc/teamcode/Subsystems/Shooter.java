package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private double buttonStartTime;

    private boolean buttonWasPressed = false;

    public Shooter(HardwareMap hardwareMap) {

        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
    }

    public void shooter(Gamepad gamepad) {

        double shooterSpeed = 1;

        if(gamepad.aWasPressed()) {

            shooterSpeed = 0;
        }
    }
}
