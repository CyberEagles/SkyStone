package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous

public class ColorSensorAuto extends LinearOpMode {
    //Declare motors and variables//

    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

    public ColorSensor sensorColorLeft;
    public DistanceSensor sensorDistanceLeft;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    double SCALE_FACTOR = 255;

    float hsvValuesLeft[] = {0F, 0F, 0F};
    final float valuesLeft[] = hsvValuesLeft;

    @Override
    public void runOpMode() {

        sensorColor = hardwareMap.get(ColorSensor.class, "rev_cds");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "rev_cds");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "rev_cdsleft");
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "rev_cdsleft");

//Tells drivers that robot is ready//
        telemetry.addData("status", "Initialized");

        waitForStart();

       telemetry.addData("First part of auto","we line up with stones");
       telemetry.update();
       sleep(2000);

       Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                (int) (sensorColorLeft.green() * SCALE_FACTOR),
                (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                hsvValuesLeft);

//        telemetry.addData("Red",sensorColor.red());
//        telemetry.addData("Green",sensorColor.green());
//        telemetry.addData("Blue",sensorColor.blue());
//        telemetry.addData("alpha",sensorColor.alpha());
//        telemetry.addData("Hue",hsvValues[0]);
//        telemetry.update();


//
//        if (LeftIsSkystone()) {
//            telemetry.addData("SKYSTONE IS", "LEFT");
//            telemetry.update();
//            sleep(5000);
//        }
//
//        else if (RightIsSkystone()) {
//            telemetry.addData("SKYSTONE IS", "MIDDLE");
//            telemetry.update();
//            sleep(5000);
//
//        }
//
//        else {
//            telemetry.addData("SKYSTONE IS", "RIGHT");
//            telemetry.update();
//            sleep(5000);
//        }

//        telemetry.addData("After Skystone", "test");
//        telemetry.update();
//        sleep(5000);

        if (SkystoneIsMiddle()){
            telemetry.addData("Skystone Detected!","MIDDLE");
            telemetry.update();
            sleep(4000);
        }
        else if (SkystoneIsRight()){
            telemetry.addData("Skystone Detected!","RIGHT");
            telemetry.update();
            sleep(4000);
        }
        else {
            telemetry.addData("NO Skystone Detected!","LEFT");
            telemetry.update();
            sleep(4000);

        }



      }

      public boolean SkystoneIsRight() {
          if ((sensorColorLeft.red() + sensorColorLeft.blue() + sensorColorLeft.green()) - (sensorColor.red() + sensorColor.blue() + sensorColor.green()) > 20) {
//              telemetry.addData("Right Sensor", "Is A Skystone");
//              telemetry.update();
//              sleep(2000);
              return true;
          }
          else {
//              telemetry.addData("Two sensors are too similar", "Both stones");
//              telemetry.update();
//              sleep(2000);
              return false;
          }
      }

    public boolean SkystoneIsMiddle() {
        if ((sensorColor.red() + sensorColor.blue() + sensorColor.green()) - (sensorColorLeft.red() + sensorColorLeft.blue() + sensorColorLeft.green()) > 20) {
//            telemetry.addData("Left Sensor", "Is A Skystone");
//            telemetry.update();
//            sleep(2000);
            return true;
        }
        else {
//            telemetry.addData("Two sensors are too similar", "Both stones");
//            telemetry.update();
//            sleep(2000);
            return false;
        }
    }


/**    public boolean RightIsSkystone() {
        if (hsvValues[0] > 70 && hsvValuesLeft[0] < 140) {
            telemetry.addData("Right Sensor", "Hue between 70 and 140. Skystone");
            telemetry.update();
            return true;
        } else {
            telemetry.addData("Right sensor", "hue is less than 70 or greater than 140. No skystone here.");
            telemetry.update();
            return false;
        }

    }

    public boolean LeftIsSkystone() {
        if (hsvValuesLeft[0] > 70 && hsvValuesLeft[0] < 140) {
                telemetry.addData("Right Sensor", "Hue between 70 and 140. Skystone");
                telemetry.update();
                return true;
            } else {
                telemetry.addData("Right sensor", "hue is less than 70 or greater than 140. No skystone here.");
                telemetry.update();
                return false;
            }
        }
*/
    }