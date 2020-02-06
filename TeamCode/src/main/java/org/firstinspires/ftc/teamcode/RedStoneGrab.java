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
        robot.goToPosition(0,-26,0.6,0,5,5, robot.BACKWARD);
        robot.turn(0.5, -75, 3, 5);
        robot.goToPosition(-3, -26, 0.6, 0, 2, 5);
        robot.skystoneGrabber.setPower(0.5);
        robot.claw.setPosition(1);
        sleep(2000);
        robot.skystoneGrabber.setPower(-0.5);
        sleep(1000);
        robot.skystoneGrabber.setPower(0);
        robot.goToPosition(-82, -26, 0.6, 0, 5, 5);
        robot.skystoneGrabber.setPower(0.5);
        sleep(700);
        robot.claw.setPosition(0);
        sleep(2000);
        robot.skystoneGrabber.setPower(-0.5);


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
