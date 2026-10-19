package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {

    private final Servo feederServo;

    public Feeder(HardwareMap hardwareMap) {
        feederServo = hardwareMap.get(Servo.class, "feederServo");
    }

    public void feeder(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            feederServo.setPosition(1);
        } else {
            feederServo.setPosition(0);
        }
    }
}