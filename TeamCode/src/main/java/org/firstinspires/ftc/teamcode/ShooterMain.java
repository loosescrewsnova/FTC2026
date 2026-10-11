package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter Only Code")
public class ShooterMain extends OpMode {

    private DcMotor shooterMotor;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        telemetry.addLine("Shooter Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        double shooterSpeed = 1.0;

        shooterMotor.setPower(shooterSpeed);

        telemetry.addData("Shooter Power", shooterSpeed);
        telemetry.update();
    }
}