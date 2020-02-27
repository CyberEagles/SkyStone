package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;
import org.firstinspires.ftc.teamcode.OdometerHardware;


@Autonomous
public class DoubleVuforiaRed extends LinearOpMode{

        OdometerHardware robot   = new OdometerHardware(this);
        @Override

        public void runOpMode () {
            robot.initDriveHardwareMap();
            robot.initVuforia();
            CameraDevice.getInstance().setFlashTorchMode(true);
            waitForStart();


            robot.goToPosition(5,-16,0.6,0,2,5,robot.BACKWARD);
            robot.turn(0.5, -80, 3, 5);
            int skystonePosition=robot.checkForSkyStone(3);
            if (skystonePosition == 1){
                telemetry.addData("Skystone Found!", "Right half of screen");
                telemetry.update();
                sleep(5000);
            }
            else if (skystonePosition == 3) {
                telemetry.addData("Skystone Found!", "Left half of screen");
                telemetry.update();
                sleep(5000);
            }
            else {
                telemetry.addData("Skystone Not Found ;(", "going for the third one");
                telemetry.update();
                sleep(5000);
            }

/**
            robot.goToPosition(-10,-24,0.6,0,2,5, robot.BACKWARD);
            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
            robot.turn(0.5, -80, 3, 5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //robot.goToPosition(-3, -26, 0.6, 0, 2, 5);
            robot.claw.setPosition(1);
            sleep(2000);
            robot.skystoneGrabber.setPower(-0.5);
            sleep(1000);
            robot.skystoneGrabber.setPower(0);
            robot.goToPosition(-50, -26, 0.8, 0, 2, 5);
//testing these 3 ^vv
            robot.skystoneGrabber.setPower(0.5);
            sleep(700);
            robot.claw.setPosition(0);
            sleep(1500);
            robot.skystoneGrabber.setPower(-0.5);
//Joel's new added fun stuff
            sleep(700);
            robot.skystoneGrabber.setPower(0);
            robot.goToPosition(-2, -26, 0.8, 0, 5, 5, robot.BACKWARD);
            robot.turn(0.5, -90, 3, 5);
            robot.skystoneGrabber.setPower(0.5);
            robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(500);
            robot.skystoneGrabber.setPower(0);
            robot.claw.setPosition(1);
            sleep(2000);
            robot.skystoneGrabber.setPower(-0.5);
            sleep(1000);
            robot.skystoneGrabber.setPower(0);
            robot.goToPosition(-55, -26, 0.8, 0, 2, 5);
//testing these 3 ^vv
            robot.skystoneGrabber.setPower(0.5);
            sleep(700);
            robot.claw.setPosition(0);
            sleep(1500);
            robot.skystoneGrabber.setPower(-0.5);
//Joel's new added fun stuff
            sleep(700);
            robot.skystoneGrabber.setPower(0);
            robot.goToPosition(-30, -26, 0.8, 0, 5, 5, robot.BACKWARD);

*/


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



