package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class RedStoneGrab extends LinearOpMode {
    OdometerHardware robot   = new OdometerHardware(this);
    @Override

    public void runOpMode () {
        robot.initDriveHardwareMap();
        waitForStart();
        robot.goToPosition(0,25,0.5,0,0.5,5);
        robot.goToPosition(2,25,0.5,0,0.5,5);
        //strafe closer to stone
        //deploy servo arm
        //strafe away from bridge
        sleep(2000);
        robot.goToPosition(50,25,0.5,0,0.5,5);
        robot.goToPosition(-8,25,0.5,0,0.5,5,robot.BACKWARD);
        robot.goToPosition(50,25,0.5,0,0.5,5,robot.FORWARD);
        robot.goToPosition(20,25,0.5,0,0.5,5,robot.BACKWARD);
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
