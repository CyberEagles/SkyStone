package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class ParkOnLineTurnRight extends LinearOpMode {
    OdometerHardware robot   = new OdometerHardware(this);
    @Override

    public void runOpMode () {
        robot.initDriveHardwareMap();
        waitForStart();
        robot.goToPosition(0,25,0.8,0, 5, 2);
        robot.goToPosition(15,25,0.8,0, 5, 2);
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
