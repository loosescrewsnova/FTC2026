package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestCS {

    NormalizedColorSensor colorSensor;


    public enum DetectedColor {
        RED,

        BLUE,

        GREEN,

        UNKNOWN
    }
    public void init(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");
    }



    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns 4 values(fractions)



            float normRed, normGreen, normBlue;
            normRed=colors.red/colors.alpha;
            normGreen=colors.green/colors.alpha;
            normBlue=colors.blue/colors.alpha;


            telemetry.addData("red",normRed);
            telemetry.addData("green",normGreen);
            telemetry.addData("blue", normBlue);

            //TODO add if statements for specific colors added
            /*
            red, green, blue
            RED =
            GREEN =
            BLUE =
             */
            return DetectedColor.UNKNOWN;


    }
}
