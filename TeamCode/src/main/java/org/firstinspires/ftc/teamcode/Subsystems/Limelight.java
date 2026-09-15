package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;


public class Limelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init(){
        limelight= hardwareMap.get(Limelight3A.class, "Limelight");
        imu = hardwareMap.get(IMU.class,"imu");

        limelight.pipelineSwitch(12); //change it based on the april tag number that occurs

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){

    }
}