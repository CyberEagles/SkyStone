package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class BlueStoneGrab extends LinearOpMode {
    OdometerHardware robot   = new OdometerHardware(this);
    @Override

    public void runOpMode () {
        robot.initDriveHardwareMap();
        waitForStart();
        robot.goToPosition(0,-25,0.6,0,2,5, robot.BACKWARD);
        robot.skystoneGrabber.setPower(0.5);
        robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(500);
        robot.skystoneGrabber.setPower(0);
        robot.turn(0.5, -75, 3, 5);
        robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.goToPosition(-3, -26, 0.6, 0, 1, 5);
        robot.claw.setPosition(1);
        sleep(2000);
        robot.skystoneGrabber.setPower(-0.5);
        sleep(1000);
        robot.skystoneGrabber.setPower(0);
        robot.goToPosition(72, -20, 0.8, 0, 2, 5, robot.BACKWARD);
//testing these 3 ^vv
        robot.goToPosition(82,-27,0.8,0,2,5,robot.BACKWARD);
        robot.turn(0.5,-90,3,5);
        robot.skystoneGrabber.setPower(0.5);
        sleep(700);
        robot.claw.setPosition(0);
        sleep(1500);
        robot.skystoneGrabber.setPower(-0.5);
//Joel's new added fun stuff
        sleep(700);
        robot.skystoneGrabber.setPower(0);
        robot.claw.setPosition(0.5);
        robot.turn(0.5,0,3,5);
        robot.goToPosition(83.5,-34,0.6,0,2,5,robot.BACKWARD);
        robot.foundation.setDirection(Servo.Direction.FORWARD);
        robot.foundation.setPosition(0);
        sleep(1000);
        robot.goToPosition(72,-10,0.8,0,3,5);
        robot.turn(0.5,-90,3,5);
        robot.goToPosition(90,-21,0.8,0,3,2,robot.BACKWARD);
        robot.foundation.setPosition(180);
        sleep(500);
        robot.goToPosition(35,-27,0.8,0,2,5);




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
