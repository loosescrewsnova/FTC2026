package org.firstinspires.ftc.teamcode.Subsystems.servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class LSServo {
    private final Servo LSServo;
    private boolean isOpen;
    private boolean isTogglePressed;
    private boolean wasTogglePressed;
    public LSServo(HardwareMap hardwareMap) {

        LSServo = hardwareMap.get(Servo.class, LSServoConstants.LS_SERVO_NAME);
        isOpen = false;
        LSServo.setPosition(LSServoConstants.CLOSED_POSITION);
    }

    public void toggle(Gamepad gamepad) {

        isTogglePressed = gamepad.right_bumper;

        // Button was just pressed (not held over from the last loop)
        if (isTogglePressed && !wasTogglePressed) {

            isOpen = !isOpen;

            if(isOpen) {
                LSServo.setPosition(LSServoConstants.OPEN_POSITION);
            } else {
                LSServo.setPosition(LSServoConstants.CLOSED_POSITION);
            }
        }

        wasTogglePressed = isTogglePressed;
    }

    public double getPosition() {
        return LSServo.getPosition();
    }

    //isOpen can be removed after testing getPosition
    public boolean isOpen() {
        return isOpen;
    }
}