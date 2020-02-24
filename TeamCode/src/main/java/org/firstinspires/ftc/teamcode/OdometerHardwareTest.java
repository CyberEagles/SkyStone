package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous

public class OdometerHardwareTest extends LinearOpMode {
    OdometerHardware robot   = new OdometerHardware(this);
    @Override

    public void runOpMode () {
        robot.initDriveHardwareMap();
        robot.initVuforia();
        waitForStart();

        //robot.goToPosition(0, 0, 0.8, 0, 5, robot.BACKWARD);
        while(opModeIsActive()){

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", robot.globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());
            telemetry.update();
            int skystonePosition=robot.checkForSkyStone(4);
            if (skystonePosition==3){
                telemetry.addData("its on the left", "tada");
                telemetry.update();
                sleep(5000);
            }
            else if (skystonePosition==1){
                telemetry.addData("it's on the right","it's tremendous it really is");
                telemetry.update();
                sleep(5000);
            }
            else if (skystonePosition==2){
                telemetry.addData("it's in the middle", "what did I tell you");
                telemetry.update();
                sleep(5000);
            }
            else if (skystonePosition==0){
                telemetry.addData("missed it","mission failed, we'll get 'em next time");
                telemetry.update();
                sleep(5000);
            }






        }

        //Stop the thread
        robot.globalPositionUpdate.stop();
    }
}
