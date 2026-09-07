package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Autonomous(name = "My First Linear OpMode", group = "Linear OpMode")
public class FirstAutonomous extends LinearOpMode {

    public void runOpMode() {

        MecanumDriveFromRyan mecanumDrive = new MecanumDriveFromRyan(hardwareMap);
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
