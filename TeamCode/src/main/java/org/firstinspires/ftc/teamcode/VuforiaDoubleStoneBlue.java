package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

@Autonomous
public class VuforiaDoubleStoneBlue extends LinearOpMode {
    OdometerHardware robot   = new OdometerHardware(this);
    @Override

    public void runOpMode () {
        double currentX;
        robot.initDriveHardwareMap();
        robot.initVuforia();
        CameraDevice.getInstance().setFlashTorchMode(true);
        waitForStart();

        robot.foundation.setPosition(0.8);
        robot.goToPosition(0,-16,0.9,0,2,3,robot.BACKWARD);
        robot.turn(0.5, -80, 3, 3);
//line up to scan^ then scan for a max of 3s v
        int skystonePosition=robot.checkForSkyStone(1.5);
        if (skystonePosition == 1){
            telemetry.addData("Skystone Found!", "Right half of screen");
            telemetry.update();

            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
//lower skystone grabber ^
    //this should be duplicated under each if loop, or above the if loop.

            robot.goToPosition(-16, -25, 0.6, 0, 1, 5, robot.FORWARD);
            robot.turn(0.6, -90, 5, 5);
            currentX = robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH;
            robot.goToPosition(currentX, -30, 0.8, 0, 3, 5, robot.STRAFELEFT);
            robot.turn(0.4, -90, 2, 2);
            //move closer to the stone ^

        }
        else if (skystonePosition == 3) {
            telemetry.addData("Skystone Found!", "Left half of screen");
            telemetry.update();

            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
//lower skystone grabber ^
            //this should be duplicated under each if loop, or above the if loop.

            robot.goToPosition(-8, -25, 0.6, 0, 1, 5, robot.FORWARD);
            robot.turn(0.6, -90, 5, 5);
            currentX = robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH;
            robot.goToPosition(currentX, -30, 0.8, 0, 3, 5, robot.STRAFELEFT);
            robot.turn(0.4, -90, 2, 2);
            //move closer to the stone ^
        }
        else {
            telemetry.addData("Skystone NOT Found!", "going for the third one");
            telemetry.update();

            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
//lower skystone grabber ^
            //this should be duplicated under each if loop, or above the if loop.

            robot.turn(0.6, -90, 5, 5);
            currentX = robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH;
            robot.goToPosition(currentX, -30, 0.8, 0, 3, 5, robot.STRAFELEFT);
            robot.turn(0.4, -90, 2, 2);
            //move closer to the stone ^

        }
        robot.claw.setPower(1);
        sleep(2500);
        robot.skystoneGrabber.setPower(-0.5);
        sleep(1000);
        robot.skystoneGrabber.setPower(0);
//grab the stone and lift it into robot ^
        robot.goToPosition(49,-25,1,0,4.5,5,robot.BACKWARD);

//drive to point closer to skybridge ^
        robot.skystoneGrabber.setPower(0.5);
        sleep(700);
//lower stone grabber with stone ^
        robot.claw.setPower(-1);
        sleep(1000);
//let go of skystone ^
        robot.skystoneGrabber.setPower(-0.5);
        sleep(700);
        robot.skystoneGrabber.setPower(0);
        robot.claw.setPower(0);
//lift skystone grabber back into robot ^
        robot.goToPosition(15,-25,1,0,4.5,5,robot.FORWARD);

        if (skystonePosition == 1){
            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
//lower skystone grabber ^
            //this should be duplicated under each if loop, or above the if loop.

            robot.goToPosition(0, -25, 0.6, 0, 1, 5, robot.FORWARD);
            robot.turn(0.6, -90, 5, 5);
            currentX = robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH;
            robot.goToPosition(currentX, -30, 0.8, 0, 3, 5, robot.STRAFELEFT);
            robot.turn(0.4, -90, 2, 2);
            //move closer to the stone ^

        }
        else if (skystonePosition == 3) {
            telemetry.addData("Skystone Found!", "Left half of screen");
            telemetry.update();

            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
//lower skystone grabber ^
            //this should be duplicated under each if loop, or above the if loop.

            robot.goToPosition(-32, -25, 1, 0, 3, 5, robot.FORWARD);
            robot.turn(0.6, -90, 5, 5);
            currentX = robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH;
            robot.goToPosition(currentX, -30, 0.8, 0, 3, 5, robot.STRAFELEFT);
            robot.turn(0.4, -75, 2, 2);
            //move closer to the stone ^
        }
        else {
            telemetry.addData("Skystone NOT Found!", "going for the third one");
            telemetry.update();

            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
//lower skystone grabber ^
            //this should be duplicated under each if loop, or above the if loop.

            robot.turn(0.6, -90, 5, 5);
            robot.goToPosition(-24, -30, 0.8, 0, 3, 5, robot.FORWARD);
            robot.turn(0.6, -90, 5, 5);
            currentX = robot.globalPositionUpdate.returnXCoordinate() / robot.COUNTS_PER_INCH;
            robot.goToPosition(currentX, -30, 0.8, 0, 3, 5, robot.STRAFELEFT);
            robot.turn(0.4, -90, 2, 5);

            //move closer to the stone ^

        }
        robot.claw.setPower(1);
        sleep(2500);
        robot.skystoneGrabber.setPower(-0.5);
        sleep(1000);
        robot.skystoneGrabber.setPower(0);
//grab the stone and lift it into robot ^
        robot.goToPosition(45,-25,1,0,4.5,5,robot.BACKWARD);

//drive to point closer to skybridge ^
        robot.skystoneGrabber.setPower(0.5);
        sleep(700);
//lower stone grabber with stone ^
        robot.claw.setPower(-1);
        sleep(1000);
//let go of skystone ^
        robot.skystoneGrabber.setPower(-0.5);
        sleep(700);
        robot.skystoneGrabber.setPower(0);
        robot.claw.setPower(0);
//lift skystone grabber back into robot ^

        robot.goToPosition(35,-27,1,0,2,5,robot.FORWARD);
        sleep(5000);
//Park on the line^

//        robot.goToPosition(0,-25,0.6,0,2,5, robot.BACKWARD);
//        robot.skystoneGrabber.setPower(0.5);
//        robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        sleep(500);
//        robot.skystoneGrabber.setPower(0);
//        robot.turn(0.5, -80, 3, 5);
//        robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.goToPosition(-5, -26, 0.6, 0, 1, 5);
//        robot.claw.setPower(1);
//        sleep(2500);
//        robot.skystoneGrabber.setPower(-0.5);
//        sleep(1000);
//        robot.skystoneGrabber.setPower(0);
//        robot.goToPosition(72, -20, 0.8, 0, 2, 5, robot.BACKWARD);
////testing these 3 ^vv
//        robot.goToPosition(82,-30,0.8,0,2,5,robot.BACKWARD);
//        robot.turn(0.5,-90,3,5);
//        robot.skystoneGrabber.setPower(0.5);
//        sleep(700);
//        robot.claw.setPower(-1);
//        sleep(2000);
//        robot.claw.setPower(0);
//        robot.skystoneGrabber.setPower(-0.5);
////Joel's new added fun stuff
//        sleep(700);
//        robot.skystoneGrabber.setPower(0);
//        robot.turn(0.5,0,3,5);
//        robot.goToPosition(83.5,-36,0.6,0,2,5,robot.BACKWARD);
//        robot.foundation.setDirection(Servo.Direction.FORWARD);
//        robot.foundation.setPosition(0);
//        sleep(1000);
//        robot.goToPosition(72,-10,0.8,0,3,5);
//        robot.turn(0.5,-90,3,5);
//        robot.goToPosition(90,-21,0.8,0,3,2,robot.BACKWARD);
//        robot.foundation.setPosition(180);
//        sleep(500);
//        robot.goToPosition(35,-27,0.8,0,2,5);
//



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
