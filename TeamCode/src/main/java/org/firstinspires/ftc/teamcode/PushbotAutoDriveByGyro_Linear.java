package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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
import java.util.Locale;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
@Autonomous

    public class PushbotAutoDriveByGyro_Linear extends LinearOpMode {

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
        static final double     DRIVE_SPEED             = 0.75;     // Nominal speed for better accuracy.
        static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

        static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
        static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable


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
//wait for start?
           //gyroTurn( TURN_SPEED,   90.0);
           gyroDrive(DRIVE_SPEED, 36, 0);
            //gyroHold(TURN_SPEED, 90, 2);
           // gyroTurn( TURN_SPEED,   -90.0);


            telemetry.addData("Path", "Complete");
            telemetry.update();
        }


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

                // Determine new target position, and pass to motor controller
                moveCounts = (int)(distance * COUNTS_PER_INCH);
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
                robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                    error = 0;
                    steer = getSteer(error, P_DRIVE_COEFF);
                    telemetry.addData("Current Position", robot.rightFrontDrive.getCurrentPosition());    //
                    telemetry.addData("Target Position", robot.rightFrontDrive.getTargetPosition());    //
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
        public void gyroTurn (  double speed, double angle) {

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
            sleep(5000);
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

    }


