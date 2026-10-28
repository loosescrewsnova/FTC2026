package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class PollenPusher {

    private final CRServo pollenPusherServo;

    private static final double PUSHING_POWER = 1.0;
    private static final double STOP_POWER = 0.0;

    public PollenPusher(HardwareMap hardwareMap) {
        pollenPusherServo = hardwareMap.get(CRServo.class, "pollenPusherServo");

        // Stop the servo initially.
        pollenPusherServo.setPower(STOP_POWER);
    }

    public void pollenPusher(Gamepad gamepad) {

        // Hold X to run the feeder.
        if (gamepad.a) {
            pollenPusherServo.setPower(PUSHING_POWER);
        } else {
            pollenPusherServo.setPower(STOP_POWER);
        }
    }

    public void stop() {
        pollenPusherServo.setPower(STOP_POWER);
    }
}
