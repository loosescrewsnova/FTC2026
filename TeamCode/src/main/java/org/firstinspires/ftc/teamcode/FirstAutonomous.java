package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@Autonomous(name = "My First Linear OpMode", group = "Linear OpMode")
public class FirstAutonomous extends LinearOpMode {

    public void runOpMode() {

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            mecanumDrive.mecanumDrive(0.8, 0, 0);
            sleep(2000);
            mecanumDrive.mecanumDrive(0, 0, 0);
        }
    }
}
