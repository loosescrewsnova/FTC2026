package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColorSensor {

    NormalizedColorSensor colorSensor;


    public enum DetectedColor {
        RED,

        BLUE,

        YELLOW,

        UNKNOWN
    }
    public void init(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        colorSensor.setGain(8);
    }



    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns 4 values(fractions)



            float normRed, normYellow, normBlue;
            normRed=colors.red/colors.alpha;
            normYellow=colors.green/colors.alpha;
            normBlue=colors.blue/colors.alpha;


            telemetry.addData("red",normRed);
            telemetry.addData("green",normYellow);
            telemetry.addData("blue", normBlue);

            //TODO add if statements for specific colors added
            /*
            red, green, blue
            RED = >.35, <.3, <.3 (This is the R-red g-green b-blue )
            Yellow = >.5, >.9, <.6
            BLUE = <.2, < .5, > .5
            */

            if (normRed > 0.35 && normYellow < 0.3 && normBlue < 0.3 ){
                return DetectedColor.RED;

            }
            else if (normBlue <0.2 && normYellow <0.5 && normBlue> 0.5){
                return DetectedColor.BLUE;

        }
            else if(normRed <0.2 && normYellow<0.5 && normBlue >0.5){
               return  DetectedColor.YELLOW;
            }
            else {
                return DetectedColor.UNKNOWN;
            }
    }
}
