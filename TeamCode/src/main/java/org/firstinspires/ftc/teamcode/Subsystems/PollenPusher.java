package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PollenPusher {

    private final Servo pollenPusherServo;

    // Servo positions (0.0 to 1.0). Tune these on the robot so the
    // servo doesn't bind at either end of its travel.
    private static final double PUSH_POSITION = 1.0;
    private static final double REST_POSITION = 0.0;

    public PollenPusher(HardwareMap hardwareMap) {
        pollenPusherServo = hardwareMap.get(Servo.class, "pollenPusherServo");
        pollenPusherServo.setPosition(REST_POSITION);
    }

    public void pollenPusher(Gamepad gamepad) {

        // Hold A (bottom button) to push; release to return to rest.
        if (gamepad.a) {
            pollenPusherServo.setPosition(PUSH_POSITION);
        } else {
            pollenPusherServo.setPosition(REST_POSITION);
        }
    }

    public void stop() {
        pollenPusherServo.setPosition(REST_POSITION);
    }
}