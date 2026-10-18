
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {

    private final Servo feederServo;

    private static final double TRIGGER_THRESHOLD = 0.1;

    // Adjust these positions to match your robot
    private static final double REST_POSITION = 0.0;
    private static final double FEED_POSITION = 1.0;

    public Feeder(HardwareMap hardwareMap) {
        feederServo = hardwareMap.get(Servo.class, "feederServo");

        feederServo.setDirection(Servo.Direction.FORWARD);
        feederServo.setPosition(REST_POSITION);
    }

    public void feeder(Gamepad gamepad) {

        if (gamepad.right_trigger > TRIGGER_THRESHOLD) {
            feederServo.setPosition(FEED_POSITION);
        } else {
            feederServo.setPosition(REST_POSITION);
        }
    }

    public double getFeederPosition() {
        return feederServo.getPosition();
    }
}