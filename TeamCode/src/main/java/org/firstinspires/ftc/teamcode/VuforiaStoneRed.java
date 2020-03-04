package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class VuforiaStoneRed extends LinearOpMode {
    OdometerHardware robot = new OdometerHardware(this);

    @Override

    public void runOpMode() {
        double currentx;
        robot.initDriveHardwareMap();
        robot.initVuforia();
        telemetry.addData("Vuforia init complete","ready to go");
        telemetry.update();
        waitForStart();
        robot.foundation.setPosition(0.8);
        robot.goToPosition(5, -16, 0.8, 0, 2, 5, robot.BACKWARD);
        robot.turn(0.8, -80, 3, 5);
        int skystonePosition = robot.checkForSkyStone(3);
        if (skystonePosition == 1) {
            telemetry.addData("Skystone Found!", "Right half of screen");
            telemetry.update();
        }
         else if (skystonePosition == 3) {
                telemetry.addData("Skystone Found!", "Left half of screen");
                telemetry.update();
            }

        else {
            telemetry.addData("Skystone Not Found ;(", "going for the third one");
            telemetry.update();
            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
            robot.goToPosition(7, -24, 0.8, 0, 3, 5, robot.BACKWARD);
            robot.goToPosition(7,-26, 0.6, 0, 1, 5, robot.STRAFELEFT);
            robot.turn(0.6, -80, 2, 5);
            robot.claw.setPower(1);
            sleep(2000);
            robot.skystoneGrabber.setPower(-0.5);
            sleep(1000);
            robot.skystoneGrabber.setPower(0);
            robot.goToPosition(-72, -16, 0.8, 0, 2, 5);
//testing these 3 ^vv
            robot.goToPosition(-82,-27,0.8,0,2,5);
            robot.turn(0.8,-90,3,5);
            robot.skystoneGrabber.setPower(0.5);
            sleep(700);
            robot.claw.setPower(-1);
            sleep(2000);
            robot.claw.setPower(0);
            robot.skystoneGrabber.setPower(-0.5);
//Joel's new added fun stuff
            sleep(700);
            robot.skystoneGrabber.setPower(0);
            robot.turn(0.8,0,3,5);
            robot.goToPosition(-82,-34,0.8,0,3,5,robot.BACKWARD);
            robot.foundation.setDirection(Servo.Direction.FORWARD);
            robot.foundation.setPosition(0);
            sleep(1000);
            robot.goToPosition(-72,-10,0.8,0,3,5);
            robot.turn(0.8,90,3,5);
            robot.goToPosition(-90,-21,0.8,0,3,2,robot.BACKWARD);
            robot.foundation.setPosition(180);
            sleep(500);
            robot.goToPosition(-30,-24,1,0,3,5);

        }



            while (opModeIsActive()) {

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

