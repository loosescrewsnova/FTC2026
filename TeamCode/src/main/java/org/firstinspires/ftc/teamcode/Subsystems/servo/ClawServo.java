package org.firstinspires.ftc.teamcode.Subsystems.servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls the claw's positional servo (a REV Smart Robot Servo), toggling it
 * between two fixed positions: open and closed.
 *
 * <p>Press the toggle button once to open the claw; press it again to close it.
 * The hardware map device name and both positions are configured in
 * {@link ClawServoConstants}.
 */
public class ClawServo {

    private final Servo clawServo;

    private boolean isOpen;
    private boolean isTogglePressed;
    private boolean wasTogglePressed;

    public ClawServo(HardwareMap hardwareMap) {

        clawServo = hardwareMap.get(Servo.class, ClawServoConstants.CLAW_SERVO_NAME);

        isOpen = false;
        clawServo.setPosition(ClawServoConstants.CLOSED_POSITION);
    }

    /**
     * Call once per loop from the OpMode. Toggles the claw between open and
     * closed each time the bound button (right bumper) is pressed.
     */
    public void update(Gamepad gamepad) {

        isTogglePressed = gamepad.right_bumper;

        // Button was just pressed (not held over from the last loop)
        if (isTogglePressed && !wasTogglePressed) {

            isOpen = !isOpen;
            clawServo.setPosition(isOpen ? ClawServoConstants.OPEN_POSITION : ClawServoConstants.CLOSED_POSITION);
        }

        wasTogglePressed = isTogglePressed;
    }

    public double getPosition() {
        return clawServo.getPosition();
    }

    public boolean isOpen() {
        return isOpen;
    }
}
