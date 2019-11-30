package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
@Autonomous

public class SkyStoneSideBlue extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();

    BNO055IMU imu;
    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.7;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     TURN_HEADING_THRESHOLD  = 7 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.75;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable
    static final double backwardsSpeed = -0.8;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();


        // make sure the gyro is calibrated before continuing
        {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();



        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d",
            telemetry.update();
        }



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        //If turning right use the separate function and set the angle to negative
        //make turn 20-23 degrees closer to zero in order to actually turn to the desired angle
//wait for start?
        gyroDrive(DRIVE_SPEED,18,0);
        gyroTurnLeft(TURN_SPEED,95);
        sleep(2000);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
//look at the first stone
                if (lookForSkystone()) {    //Refine intake position
                    //grab it
                    telemetry.addData("Skystone Detected! Position","1");
                    telemetry.update();
                    gyroDriveBackwards(DRIVE_SPEED,13,0);
                    gyroTurnRight(TURN_SPEED,10);
                    robot.rightIntake.setPower(0.75);
                    robot.leftIntake.setPower(-0.75);
                    gyroDrive(DRIVE_SPEED/2,20,0);
                    gyroDriveBackwards(DRIVE_SPEED,15,0);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroTurnLeft(TURN_SPEED,95);
                    gyroDrive(DRIVE_SPEED,65,0);
                    robot.rightIntake.setPower(-0.75);
                    robot.leftIntake.setPower(0.75);
                    gyroTurnRight(TURN_SPEED, -95);
                    gyroDrive(DRIVE_SPEED, 60, 0);
                    robot.rightIntake.setPower(0.75);
                    robot.leftIntake.setPower(-0.75);
                    //Adjust this next turn
                    gyroTurnLeft(TURN_SPEED, -40);
                    gyroDrive(DRIVE_SPEED/2, 25, 0);
                    gyroDriveBackwards(DRIVE_SPEED, 20, 0);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroTurnLeft(TURN_SPEED, 95);
                    gyroDrive(DRIVE_SPEED, 63, 0);
                    robot.rightIntake.setPower(-0.75);
                    robot.leftIntake.setPower(0.75);
                    sleep(2000);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroDriveBackwards(DRIVE_SPEED,15,0);
                    sleep(999999999);

                } else {
                    //drive to next one
                    telemetry.addData("Skystone Not Found! Continue to Position","2");
                    telemetry.update();
                    gyroDrive(DRIVE_SPEED,8,0);
                }
                //If other paths not working disable TensorFlow and use the one above
//look again at the next stone
                sleep(3000);
                if (lookForSkystone()) {
                    //grab it
                    telemetry.addData("Skystone Detected! Position","2");
                    telemetry.update();
                    gyroDriveBackwards(DRIVE_SPEED,13,0);
                    gyroTurnRight(TURN_SPEED,10);
                    robot.rightIntake.setPower(0.75);
                    robot.leftIntake.setPower(-0.75);
                    gyroDrive(DRIVE_SPEED/2,20,0);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroDriveBackwards(DRIVE_SPEED,15,0);
                    gyroTurnLeft(TURN_SPEED,95);
                    gyroDrive(DRIVE_SPEED,60,0);
                    robot.rightIntake.setPower(-0.75);
                    robot.leftIntake.setPower(0.75);
                    gyroDriveBackwards(DRIVE_SPEED,55,0);
//                    gyroTurnRight(TURN_SPEED, -95);
//                    gyroDrive(DRIVE_SPEED, 55, 0);
                    robot.rightIntake.setPower(0.75);
                    robot.leftIntake.setPower(-0.75);
                    gyroTurnRight(TURN_SPEED, -40);
                    gyroDrive(DRIVE_SPEED, 25, 0);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroDriveBackwards(DRIVE_SPEED, 20, 0);
                    gyroTurnLeft(TURN_SPEED, 95);
                    gyroDrive(DRIVE_SPEED, 58, 0);
                    robot.rightIntake.setPower(-0.75);
                    robot.leftIntake.setPower(0.75);
                    sleep(2000);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroDriveBackwards(DRIVE_SPEED,15,0);
                    sleep(999999999);

                } else {
                    //drive to the next one and grab it
                    telemetry.addData("Skystone Not Found! Continue to Position","3");
                    telemetry.update();
                    gyroDriveBackwards(DRIVE_SPEED,8,0);
                    gyroTurnRight(TURN_SPEED,10);
                    robot.rightIntake.setPower(0.75);
                    robot.leftIntake.setPower(-0.75);
                    gyroDrive(DRIVE_SPEED/2,20,0);
                    gyroDriveBackwards(DRIVE_SPEED,15,0);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroTurnRight(TURN_SPEED,-95);
                    gyroDrive(DRIVE_SPEED,60,0);
                    robot.rightIntake.setPower(-0.75);
                    robot.leftIntake.setPower(0.75);
                    gyroTurnLeft(TURN_SPEED, 95);
                    gyroDrive(DRIVE_SPEED, 70, 0);
                    robot.rightIntake.setPower(0.75);
                    robot.leftIntake.setPower(-0.75);
                    gyroTurnRight(TURN_SPEED, 50);
                    gyroDrive(DRIVE_SPEED, 25, 0);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroDriveBackwards(DRIVE_SPEED, 20, 0);
                    gyroTurnRight(TURN_SPEED, -95);
                    gyroDrive(DRIVE_SPEED, 63, 0);
                    robot.rightIntake.setPower(-0.75);
                    robot.leftIntake.setPower(0.75);
                    sleep(2000);
                    robot.rightIntake.setPower(0);
                    robot.leftIntake.setPower(0);
                    gyroDriveBackwards(DRIVE_SPEED,15,0);
                    sleep(999999999);
                }
                sleep(999999999);
            }
        }



    }

//            gyroDrive(DRIVE_SPEED, 1, 0);
//        gyroTurnLeft( TURN_SPEED,   90);
//        gyroDriveBackwards (backwardsSpeed, 12, 0);
//        gyroDrive(DRIVE_SPEED,12,0);
    //gyroHold(TURN_SPEED, 90, 2);
    // gyroTurn( TURN_SPEED,   -90.0);
    //sleep(5000000);

//            telemetry.addData("Path", "Complete");
//////            telemetry.update();



    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            if (distance>=36){distance=distance-14.0+11.0;}
            if (24<=distance) {distance=distance-11.0;}
            else  {distance=distance-7.0;}

            // Determine new target position, and pass to motor controller
            moveCounts = (int)((distance) * COUNTS_PER_INCH);
            //newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
            //newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
//                newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            // robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
//                robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            // robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
//                robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set Target and Turn On RUN_TO_POSITION



            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftBackDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (robot.rightFrontDrive.getCurrentPosition()<newRightFrontTarget)){

                //             (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = 0; //getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                telemetry.addData("Current Position", robot.rightFrontDrive.getCurrentPosition());    //
                telemetry.addData("Target Position",newRightFrontTarget);    //
                telemetry.addData("Error", error);
                telemetry.addData("Speed", speed);
                telemetry.update();

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftBackDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);
                robot.rightBackDrive.setPower(rightSpeed);

                // Display drive status for the driver.

            }

            // Stop all motion;
            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroDriveBackwards ( double speed,
                                     double distance,
                                     double angle) {

        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            if (distance>=36){distance=distance-14.0+11.0;}
            if (24<=distance) {distance=distance-11.0;}
            else  {distance=distance-7.0;}

            // Determine new target position, and pass to motor controller
            moveCounts = (int)((distance) * COUNTS_PER_INCH);
            //newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
            //newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
//                newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            // robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
//                robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            // robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
//                robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set Target and Turn On RUN_TO_POSITION



            // start motion.
            speed = Range.clip(Math.abs(speed), -1.0, 1.0);
            robot.leftBackDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (-robot.rightFrontDrive.getCurrentPosition()>-newRightFrontTarget)){

                //             (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = 0; //getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                telemetry.addData("Current Position", robot.rightFrontDrive.getCurrentPosition());    //
                telemetry.addData("Target Position",newRightFrontTarget);    //
                telemetry.addData("Error", error);
                telemetry.addData("Speed", speed);
                telemetry.update();

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftBackDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);
                robot.rightBackDrive.setPower(rightSpeed);

                // Display drive status for the driver.

            }

            // Stop all motion;
            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurnLeft (  double speed, double angle) {
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !TurnonHeading(speed, angle-20, P_TURN_COEFF)) {
            robot.leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            telemetry.addData("motors turning on",0);
            robot.leftFrontDrive.setPower(-TURN_SPEED);
            robot.rightFrontDrive.setPower(TURN_SPEED);
            robot.rightBackDrive.setPower(TURN_SPEED);
            robot.leftBackDrive.setPower(-TURN_SPEED);
        }
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        sleep(500);
    }

    public void gyroTurnRight (  double speed, double angle) {
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !TurnonHeading(speed, angle+20, P_TURN_COEFF)) {
            robot.leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            telemetry.addData("motors turning on",0);
            robot.leftFrontDrive.setPower(TURN_SPEED);
            robot.rightFrontDrive.setPower(-TURN_SPEED);
            robot.rightBackDrive.setPower(-TURN_SPEED);
            robot.leftBackDrive.setPower(TURN_SPEED);
        }
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        sleep(500);
    }


    /** public void gyroTurn (  double speed, double angle) {

     // keep looping while we are still active, and not on heading.
     while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
     // Update telemetry & Allow time for other processes to run.
     telemetry.update();
     //robot.leftFrontDrive.setPower(-1);
     //robot.rightFrontDrive.setPower(1);
     //robot.rightBackDrive.setPower(1);
     //robot.leftBackDrive.setPower(-1);
     }
     robot.leftFrontDrive.setPower(0);
     robot.rightFrontDrive.setPower(0);
     robot.rightBackDrive.setPower(0);
     robot.leftBackDrive.setPower(0);
     }

     /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = -speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontDrive.setPower(-leftSpeed);
        robot.rightFrontDrive.setPower(-rightSpeed);
        robot.leftBackDrive.setPower(-leftSpeed);
        robot.rightBackDrive.setPower(-rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    boolean TurnonHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= TURN_HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = -speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontDrive.setPower(-leftSpeed);
        robot.rightFrontDrive.setPower(-rightSpeed);
        robot.leftBackDrive.setPower(-leftSpeed);
        robot.rightBackDrive.setPower(-rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    /** boolean onHeading(double speed, double angle, double PCoeff) {
     double   error ;
     double   steer ;
     boolean  onTarget = false ;
     double leftSpeed;
     double rightSpeed;

     // determine turn power based on +/- error
     error = getError(angle);

     if (Math.abs(error) <= HEADING_THRESHOLD) {
     steer = 0.0;
     leftSpeed  = 0.0;
     rightSpeed = 0.0;
     onTarget = true;
     }
     else {
     steer = getSteer(error, PCoeff);
     rightSpeed  = -speed * steer;
     leftSpeed   = -rightSpeed;
     }

     // Send desired speeds to motors.
     robot.leftFrontDrive.setPower(-leftSpeed);
     robot.rightFrontDrive.setPower(-rightSpeed);
     robot.leftBackDrive.setPower(-leftSpeed);
     robot.rightBackDrive.setPower(-rightSpeed);

     // Display it for the driver.
     telemetry.addData("Target", "%5.2f", angle);
     telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
     telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

     return onTarget;
     }

     /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        Orientation angles;
        // calculate error in -179 to +180 range  (
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    private boolean lookForSkystone() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                boolean foundSkyStone = false;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    if (recognition.getLabel() == "Skystone") {
                        foundSkyStone = true;
                    }
                }
                telemetry.update();
                return foundSkyStone;
            }
        }
        return false;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}


