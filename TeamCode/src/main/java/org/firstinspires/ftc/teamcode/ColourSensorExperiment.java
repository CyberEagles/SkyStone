package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous

public class ColourSensorExperiment extends LinearOpMode {
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


//Tells drivers that robot is ready//
        telemetry.addData("status", "Initialized");

        waitForStart();

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        int ColourCondition = (sensorColor.red() / sensorColor.blue()) * (sensorColor.green() / sensorColor.blue());

        telemetry.addData("Before color sensor stuff", "Hi");
        telemetry.update();
        sleep(3000);
        if (ColourCondition<= 2 ) {
            telemetry.addData("Its A Skystone", "Wow");
            telemetry.update();
            sleep(10000);

        } else {
            telemetry.addData("Its Not a Skystone", "mission failed we'll get 'em next time");
            telemetry.update();
            sleep(10000);

        }



        telemetry.addData("After Skystone", "test");
        telemetry.update();
        sleep(5000);

    }


}



