package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {

    private final DcMotor feederMotor;
    private double feederPower = 0;

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double FEED_POWER = 1.0;

    public Feeder(HardwareMap hardwareMap) {
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        // Flip to FORWARD if the feeder spins the wrong way
        feederMotor.setDirection(DcMotor.Direction.REVERSE);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void feeder(Gamepad gamepad) {

        // Hold right trigger to run the feeder, release to stop
        feederPower = (gamepad.right_trigger > TRIGGER_THRESHOLD) ? FEED_POWER : 0;
        feederMotor.setPower(feederPower);
    }

    public double getFeederPower() {
        return feederPower;
    }
}