package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Feeder {

    private final CRServo feederServo;

    private static final double FEED_POWER = 1.0;
    private static final double STOP_POWER = 0.0;

    public Feeder(HardwareMap hardwareMap) {
        feederServo = hardwareMap.get(CRServo.class, "feederServo");

        // Stop the servo initially.
        feederServo.setPower(STOP_POWER);
    }

    public void feeder(Gamepad gamepad) {

        feederServo.setPower(0);

        while(gamepad.x) {
            feederServo.setPower(1);
        }
    }

    public void stop() {
        feederServo.setPower(STOP_POWER);
    }

    public Object getFeederPower() {
        return null;
    }
}
