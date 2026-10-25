package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {

    private final CRServo feederServo;

    private static final double FEED_POWER = 1.0;
    private static final double STOP_POWER = 0.0;

    public Feeder(HardwareMap hardwareMap) {
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        feederServo.setPower(STOP_POWER);
    }

    public void feeder(Gamepad gamepad) {

        // Hold X (left button) to run the feeder; release to stop.
        if (gamepad.x) {
            feederServo.setPower(FEED_POWER);
        } else {
            feederServo.setPower(STOP_POWER);
        }
    }

    public void stop() {
        feederServo.setPower(STOP_POWER);
    }

    public static double getFeedPower() {
        return FEED_POWER;
    }
}