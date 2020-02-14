package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
@Disabled
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

       telemetry.addData("Before color sensor stuff","Hi");
       telemetry.update();
       sleep(3000);

       Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                (int) (sensorColorLeft.green() * SCALE_FACTOR),
                (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                hsvValuesLeft);


        if (LeftIsSkystone()) {
            telemetry.addData("SKYSTONE IS", "LEFT");
            telemetry.update();
            sleep(5000);
        }

        else if (RightIsSkystone()) {
            telemetry.addData("SKYSTONE IS", "MIDDLE");
            telemetry.update();
            sleep(5000);

        }

        else {
            telemetry.addData("SKYSTONE IS", "RIGHT");
            telemetry.update();
            sleep(5000);
        }

        telemetry.addData("After Skystone", "test");
        telemetry.update();
        sleep(5000);

      }

    public boolean RightIsSkystone() {
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
    }