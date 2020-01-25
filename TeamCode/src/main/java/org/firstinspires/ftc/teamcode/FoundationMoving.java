package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "FoundationMoving")
public class FoundationMoving extends LinearOpMode {
    //Drive motors
    DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;



    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();



        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
//        globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);




        //      globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

//        globalPositionUpdate.reverseLeftEncoder();

        goToPosition(0,30,0.2,0,1);
        goToPosition(0,0, 0.2, 0, 1);

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());


            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();


        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {

        double distanceToXTarget = targetXPosition*COUNTS_PER_INCH - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition*COUNTS_PER_INCH - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() && distance > allowableDistanceError*COUNTS_PER_INCH) {

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            distanceToXTarget = targetXPosition*COUNTS_PER_INCH - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition*COUNTS_PER_INCH - globalPositionUpdate.returnYCoordinate();


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            if (robotMovementAngle <0){
                robotMovementAngle += 360;
            }

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            double orientationAngle = globalPositionUpdate.returnOrientation();

            if (orientationAngle < 0){
                orientationAngle += 360;
            }

            double angleDifference = robotMovementAngle - orientationAngle;

            while (angleDifference > 180)  angleDifference -= 360;
            while (angleDifference <= -180) angleDifference += 360;

            if (Math.abs(angleDifference) > 10) {
                if ( angleDifference > 0) {
                    //turn right
                    leftFrontDrive.setPower(robotPower);
                    rightFrontDrive.setPower(robotPower);
                    leftBackDrive.setPower(robotPower);
                    rightBackDrive.setPower(robotPower);

                    telemetry.addData("Turning right", "now");
                    telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    telemetry.addData("Orientation (Degrees)", orientationAngle);
                    telemetry.addData("Robot Movement Angle", robotMovementAngle);


                    telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    telemetry.update();

                }
                else if (angleDifference< 0) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(-robotPower);


                    telemetry.addData("Turning left", "now");
                    telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    telemetry.addData("Orientation (Degrees)", orientationAngle);
                    telemetry.addData("Robot Movement Angle", robotMovementAngle);


                    telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    telemetry.update();

                    //turn left
                }


            }
            else {
                //drive to target
                telemetry.addData("drive","This is the else loop");
                telemetry.addData("X Position Inches", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
                telemetry.addData("Y Position Inches",globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
                telemetry.addData("Distance to X Target", distanceToXTarget/COUNTS_PER_INCH);
                telemetry.addData("Distance To Y Target", distanceToYTarget/COUNTS_PER_INCH);
                telemetry.update();
                leftFrontDrive.setPower(robotPower);
                rightFrontDrive.setPower(-robotPower);
                leftBackDrive.setPower(robotPower);
                rightBackDrive.setPower(-robotPower);
            }

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        telemetry.addData("Distance to X Target", distanceToXTarget/COUNTS_PER_INCH);
        telemetry.addData("Distance To Y Target", distanceToYTarget/COUNTS_PER_INCH);
        telemetry.update();


    }


    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        rightFrontDrive = hardwareMap.dcMotor.get(rfName);
        rightBackDrive = hardwareMap.dcMotor.get(rbName);
        leftFrontDrive = hardwareMap.dcMotor.get(lfName);
        leftBackDrive = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
