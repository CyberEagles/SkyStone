package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import android.hardware.camera2.CameraDevice;



import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


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
    DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive, skystoneGrabber, craneMotor;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    public CRServo clamp = null;             //grabbing stones
    public CRServo claw = null;              //skystone Grabber pinch servo, Part 2
    public CRServo stoneRotator = null;    //once stone is grabbed move outside robot
    public Servo foundation = null;

    final double COUNTS_PER_INCH = 307.699557;
    final int FORWARD = 1;
    final int BACKWARD = 2;
    final int STRAFELEFT = 3;
    final int STRAFERIGHT = 4;


    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rfName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION




    //      globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

//        globalPositionUpdate.reverseLeftEncoder();
    public void initVuforia (){
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line


        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        targetsSkyStone.activate();
        opMode.telemetry.addData("Vuforia", "initialized");
        opMode.telemetry.update();
    }

    public int checkForSkyStone(double timeoutS) {
        runtime.reset();

        while (!opMode.isStopRequested() && runtime.seconds() < timeoutS) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {

                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && (trackable.getName() == "Stone Target")) {
                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;


                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                double X = translation.get(0) / mmPerInch;
                double Y = translation.get(1) / mmPerInch;
                double Z = translation.get(2) / mmPerInch;

//before detection
                if (Y > 0) {
                    opMode.telemetry.addData("Y > 0", "RIGHT half of screen");
                    opMode.telemetry.update();
                    return 1;
                    //

                } else if (Y < 0) {
                    opMode.telemetry.addData("Y < 0", "LEFT half of screen");
                    opMode.telemetry.update();
                    return 3;

                } else {
                    opMode.telemetry.addData("Between 3 and -3", "MIDDLE of screen");
                    opMode.telemetry.update();
                    return 2;

                }


            } else {
                opMode.telemetry.addData("Visible Target", "none");

            }
            opMode.telemetry.update();

        }
return 0;
    }

    public void turn (double robotPower, double desiredRobotOrientation, double allowableAngleError, double timeoutS) {
        runtime.reset();

        double orientationAngle = globalPositionUpdate.returnOrientation();

        if (orientationAngle < 0){
            orientationAngle += 360;
        }

        double angleDifference = desiredRobotOrientation - orientationAngle;

        while (angleDifference > 180)  angleDifference -= 360;
        while (angleDifference <= -180) angleDifference += 360;

        while (opMode.opModeIsActive() && Math.abs(angleDifference) > allowableAngleError && (runtime.seconds() < timeoutS)) {



            orientationAngle = globalPositionUpdate.returnOrientation();

            if (orientationAngle < 0){
                orientationAngle += 360;
            }

            angleDifference = desiredRobotOrientation - orientationAngle;

            while (angleDifference > 180)  angleDifference -= 360;
            while (angleDifference <= -180) angleDifference += 360;

            if (Math.abs(angleDifference) > allowableAngleError) {

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


                    opMode.telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    opMode. telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    opMode. telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

                    opMode.telemetry.update();

                    //turn left
                }

            }



        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);


    }





    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double timeoutS) {
        goToPosition(targetXPosition, targetYPosition, robotPower, desiredRobotOrientation, allowableDistanceError, timeoutS, FORWARD);
    }
    // To fix Strafe create second method

    public void initDriveHardwareMap(){
        String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
        String vlEncoderName = rfName, vrEncoderName = lfName, hEncoderName = lbName;

        rightFrontDrive = opMode.hardwareMap.dcMotor.get(rfName);
        rightBackDrive = opMode.hardwareMap.dcMotor.get(rbName);
        leftFrontDrive = opMode.hardwareMap.dcMotor.get(lfName);
        leftBackDrive = opMode.hardwareMap.dcMotor.get(lbName);
        skystoneGrabber = opMode.hardwareMap.dcMotor.get("skystone");
        craneMotor = opMode.hardwareMap.get (DcMotor.class, "crane");

        claw = opMode.hardwareMap.crservo.get("claw");
        clamp = opMode.hardwareMap.crservo.get("clamp");
        stoneRotator = opMode.hardwareMap.crservo.get("stone_rotator");
        foundation = opMode.hardwareMap.servo.get ("foundation");


        verticalLeft = opMode.hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = opMode.hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = opMode.hardwareMap.dcMotor.get(hEncoderName);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        craneMotor.setDirection(DcMotorSimple.Direction.FORWARD);



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

            if (direction == STRAFERIGHT) {
                robotMovementAngle +=270;
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
                opMode.telemetry.addData("drive", "This is the else loop");
                opMode.telemetry.addData("X Position Inches", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                opMode.telemetry.addData("Y Position Inches", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                opMode.telemetry.addData("Distance to X Target", distanceToXTarget / COUNTS_PER_INCH);
                opMode.telemetry.addData("Distance To Y Target", distanceToYTarget / COUNTS_PER_INCH);
                opMode.telemetry.update();
                if (direction == FORWARD) {
                    leftFrontDrive.setPower(robotPower);
                    rightFrontDrive.setPower(-robotPower);
                    leftBackDrive.setPower(robotPower);
                    rightBackDrive.setPower(-robotPower);
                } else if (direction == BACKWARD) {
                    leftFrontDrive.setPower(-robotPower);
                    rightFrontDrive.setPower(robotPower);
                    leftBackDrive.setPower(-robotPower);
                    rightBackDrive.setPower(robotPower);
                } else if (direction == STRAFELEFT) {
                    leftFrontDrive.setPower(-robotPower * 0.75);
                    rightFrontDrive.setPower(-robotPower * 0.75);
                    leftBackDrive.setPower(robotPower * 0.75);
                    rightBackDrive.setPower(robotPower * 0.8);
                } else if (direction == STRAFERIGHT) {
                    leftFrontDrive.setPower(robotPower * 0.75);
                    rightFrontDrive.setPower(robotPower * 0.75);
                    leftBackDrive.setPower(-robotPower * 0.75);
                    rightBackDrive.setPower(-robotPower * 0.8);

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
}
