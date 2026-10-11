package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private final DcMotor shooterMotor;

    // Speed range and how long a full sweep takes while the d-pad is held
    private static final double MIN_SPEED = 0.0;
    private static final double MAX_SPEED = 1.0;
    private static final double FULL_SWEEP_SECONDS = 10.0;

    // Units of power per second: (1.0 - 0.0) / 10 s = 0.1 per second
    private static final double RAMP_RATE =
            (MAX_SPEED - MIN_SPEED) / FULL_SWEEP_SECONDS;

    // Safety cap on dt so a long hiccup between loops can't cause a big jump
    private static final double MAX_DT = 0.1;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime;

    private double shooterSpeed = MIN_SPEED;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        // Flip to REVERSE if the shooter spins the wrong way
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        lastTime = timer.seconds();
    }

    public void shooter(Gamepad gamepad) {

        double shooterSpeed = 1;

        if(gamepad.aWasPressed()) {

            shooterSpeed = 0;
        }
    }
}
