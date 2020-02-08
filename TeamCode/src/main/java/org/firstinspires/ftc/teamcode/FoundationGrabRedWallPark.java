package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class FoundationGrabRedWallPark extends LinearOpMode {
    OdometerHardware robot   = new OdometerHardware(this);
    @Override

    public void runOpMode () {
        robot.initDriveHardwareMap();
        waitForStart();
        robot.foundation.setDirection(Servo.Direction.FORWARD);
        robot.goToPosition(0,-6,0.8,0,3,5,robot.BACKWARD);
        robot.goToPosition(-15,-25,0.8,0,3,5,robot.BACKWARD);
        robot.turn(0.5,-30,10,5);
        robot.goToPosition(-15,-35,0.5,0,3,5,robot.BACKWARD);
        robot.foundation.setPosition(0);
        sleep(2000);
        robot.goToPosition(-15,-25,0.8,0,2,5);
        robot.goToPosition(10,-15,0.8,0,3,5,robot.FORWARD);
        robot.turn(0.5,90,10,5);
        robot.goToPosition(-20,-12,0.8,0,3,2,robot.BACKWARD);
        robot.foundation.setPosition(180);
        sleep(1000);
        robot.goToPosition(32,-5,0.8,0,2,5,robot.BACKWARD);
        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH);
            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate() / robot.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", robot.globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());



            telemetry.update();


        }

        //Stop the thread
        robot.globalPositionUpdate.stop();
    }
}
