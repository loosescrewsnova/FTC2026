package org.firstinspires.ftc.teamcode.Subsystems.servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class LSServo {
    private final Servo clawServo;
    private boolean isOpen;
    private boolean isTogglePressed;
    private boolean wasTogglePressed;
    public LSServo(HardwareMap hardwareMap) {

        clawServo = hardwareMap.get(Servo.class, ClawServoConstants.CLAW_SERVO_NAME);
        isOpen = false;
        clawServo.setPosition(ClawServoConstants.CLOSED_POSITION);
    }

    public void toggle(Gamepad gamepad) {

        isTogglePressed = gamepad.right_bumper;

        // Button was just pressed (not held over from the last loop)
        if (isTogglePressed && !wasTogglePressed) {

            isOpen = !isOpen;

            if(isOpen) {
                clawServo.setPosition(ClawServoConstants.OPEN_POSITION);
            } else {
                clawServo.setPosition(ClawServoConstants.CLOSED_POSITION);
            }
        }

        wasTogglePressed = isTogglePressed;
    }

    public double getPosition() {
        return clawServo.getPosition();
    }

    //isOpen can be removed after testing getPosition
    public boolean isOpen() {
        return isOpen;
    }
}