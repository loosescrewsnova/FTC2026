package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PollenPusher {

    private final Servo pollenPusherServo;

    // Servo positions (0.0 to 1.0). Tune these on the robot so the
    // servo doesn't bind at either end of its travel.
    private static final double PUSH_POSITION = 0.9;
    private static final double REST_POSITION = 0.3;

    private boolean isPushing = false;
    private boolean previousButtonPressed = false;

    public PollenPusher(HardwareMap hardwareMap) {
        pollenPusherServo = hardwareMap.get(Servo.class, "pollenPusherServo");
        pollenPusherServo.setPosition(REST_POSITION);
    }

    public void pollenPusher(Gamepad gamepad) {

        // Click once to push and click again to return to rest.
        boolean buttonPressed = gamepad.left_trigger_pressed;
        if (buttonPressed && !previousButtonPressed) {
            isPushing = !isPushing;
        }
        previousButtonPressed = buttonPressed;

        if (isPushing) {
            pollenPusherServo.setPosition(PUSH_POSITION);
        } else {
            pollenPusherServo.setPosition(REST_POSITION);
        }
    }

    public void stop() {
        isPushing = false;
        previousButtonPressed = false;
        pollenPusherServo.setPosition(REST_POSITION);
    }
}
