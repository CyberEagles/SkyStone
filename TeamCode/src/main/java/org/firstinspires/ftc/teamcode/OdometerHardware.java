package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */

public class OdometerHardware {
    //Drive motors
    LinearOpMode opMode = null;
    public ElapsedTime     runtime = new ElapsedTime();
    public OdometerHardware (LinearOpMode opMode){
        this.opMode= opMode;}
    DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;
    final int FORWARD = 1;
    final int BACKWARD = 2;
    final int STRAFELEFT = 3;
    final int STRAFERIGHT = 4;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    public Servo foundation = null;
    public Servo skystoneGrabber = null;
    public CRServo stoneRotator = null;
    public Servo clamp = null;



    OdometryGlobalCoordinatePosition globalPositionUpdate;





        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION




        //      globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

//        globalPositionUpdate.reverseLeftEncoder();






    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double timeoutS) {
        goToPosition(targetXPosition, targetYPosition, robotPower, desiredRobotOrientation, allowableDistanceError, timeoutS, FORWARD);
    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double timeoutS, int direction) {

        double distanceToXTarget = targetXPosition*COUNTS_PER_INCH - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition*COUNTS_PER_INCH - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        runtime.reset();

        while (opMode.opModeIsActive() && distance > allowableDistanceError*COUNTS_PER_INCH && (runtime.seconds() < timeoutS)) {

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            distanceToXTarget = targetXPosition*COUNTS_PER_INCH - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition*COUNTS_PER_INCH - globalPositionUpdate.returnYCoordinate();


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            if (direction == BACKWARD) {
                robotMovementAngle +=180;
            }

            if (direction == STRAFELEFT) {
                robotMovementAngle +=90;
            }

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

                    opMode.telemetry.addData("Turning right", "now");
                    opMode.telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Orientation (Degrees)", orientationAngle);
                    opMode.telemetry.addData("Robot Movement Angle", robotMovementAngle);


                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode.telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode.telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                }
                else if (angleDifference< 0) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(-robotPower);


                    opMode. telemetry.addData("Turning left", "now");
                    opMode. telemetry.addData("X Position (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Y Position (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                    opMode.telemetry.addData("Orientation (Degrees)", orientationAngle);
                    opMode.telemetry.addData("Robot Movement Angle", robotMovementAngle);


                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode. telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode. telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                    //turn left
                }

            }

            else {
                //drive to target
                opMode.telemetry.addData("drive","This is the else loop");
                opMode.telemetry.addData("X Position Inches", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
                opMode.telemetry.addData("Y Position Inches",globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
                opMode.telemetry.addData("Distance to X Target", distanceToXTarget/COUNTS_PER_INCH);
                opMode.telemetry.addData("Distance To Y Target", distanceToYTarget/COUNTS_PER_INCH);
                opMode.telemetry.update();
                if (direction == FORWARD) {
                    leftFrontDrive.setPower(robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(robotPower);
                    rightBackDrive.setPower(-robotPower);
                }
                else if (direction == BACKWARD) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(robotPower);
                }
                else if (direction == STRAFELEFT) {
                    leftFrontDrive.setPower(-robotPower*0.75);
                    rightFrontDrive.setPower(-robotPower*0.75);
                    leftBackDrive.setPower(robotPower*0.75);
                    rightBackDrive.setPower(robotPower*0.8);
                }
            }


        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        opMode.telemetry.addData("Distance to X Target", distanceToXTarget/COUNTS_PER_INCH);
        opMode.telemetry.addData("Distance To Y Target", distanceToYTarget/COUNTS_PER_INCH);
        opMode.telemetry.update();


    }
    // To fix Strafe create second method


    public void initDriveHardwareMap(){
        String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
        String vlEncoderName = rfName, vrEncoderName = lfName, hEncoderName = lbName;

        rightFrontDrive = opMode.hardwareMap.dcMotor.get(rfName);
        rightBackDrive = opMode.hardwareMap.dcMotor.get(rbName);
        leftFrontDrive = opMode.hardwareMap.dcMotor.get(lfName);
        leftBackDrive = opMode.hardwareMap.dcMotor.get(lbName);

        verticalLeft = opMode.hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = opMode.hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = opMode.hardwareMap.dcMotor.get(hEncoderName);


        foundation = opMode.hardwareMap.servo.get("foundation");
        stoneRotator = opMode.hardwareMap.crservo.get("stone_rotator");
        skystoneGrabber = opMode.hardwareMap.servo.get("skystone");
        clamp = opMode.hardwareMap.servo.get("grabber");


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



        opMode.telemetry.addData("Status", "Init Complete");
        opMode.telemetry.update();




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

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
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
