/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous

public class OdometerTesting extends LinearOpMode {






    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.24;     // This is < 1.0 if geared UP //0.75 for chassis, x for susan
    static final double     LIFT_GEAR_REDUCTION     = 2;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     NO_WHEEL                = 0.5;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)*0.6;
    static final double     COUNTS_PER_LIFT_INCH    = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION) / (NO_WHEEL * 3.1415);
    static final double     DRIVE_SPEED             = 0.5 ;
    static final double     TURN_SPEED              = 0.5;
    static final double     LIFT_SPEED              = 1;

    @Override
    public void runOpMode() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // robot.intakeFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.dropOffMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.intakeFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.dropOffMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder resetwx
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition());


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("keeping the phones awake","part 1");
        telemetry.update();
// put the start of every auto here

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

//        encoderDrive(DRIVE_SPEED,  -48,  -48, 48, 48, 0,5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        // -,-,+,+ = forward
        // +,+,+,+ = turn left 22.5 for 90 degrees and 12 ish for 45
        // +,-,+,- = strafe left. still a little sketchy though


        telemetry.addData("keeping the phones awake","part 2");
        telemetry.update();
        /**LIFT right below*/

        DriveBackwards(DRIVE_SPEED,12,5);




        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void DriveForward (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(Math.abs((speed*1)));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {


                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
//                        robot.leftFrontDrive.getCurrentPosition(),
//                        robot.rightFrontDrive.getCurrentPosition(),
//                        robot.leftBackDrive.getCurrentPosition(),
//                        robot.rightBackDrive.getCurrentPosition());
//                telemetry.update();
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);



//                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void TurnLeftNew (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(-Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(-Math.abs((speed*1)));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
//                        robot.leftFrontDrive.getCurrentPosition(),
//                        robot.rightFrontDrive.getCurrentPosition(),
//                        robot.leftBackDrive.getCurrentPosition(),
//                        robot.rightBackDrive.getCurrentPosition());
//                telemetry.update();
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);



//                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void TurnRightNew (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(-Math.abs(speed));
            robot.rightFrontDrive.setPower(-Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(Math.abs((speed*1)));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
//                        robot.leftFrontDrive.getCurrentPosition(),
//                        robot.rightFrontDrive.getCurrentPosition(),
//                        robot.leftBackDrive.getCurrentPosition(),
//                        robot.rightBackDrive.getCurrentPosition());
//                telemetry.update();
            }

            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftBackDrive.setPower(0);



//                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
    /**
     public void TurnLeft (double speed, double turnInches, double timeoutS) {
     int newLeftBackTarget;
     int newRightBackTarget;
     int newRightFrontTarget;
     int newLeftFrontTarget;
     int newLiftInches;

     // Ensure that the opmode is still active
     if (opModeIsActive()) {

     // Determine new target position, and pass to motor controller
     newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int) (turnInches * COUNTS_PER_INCH);
     newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int) (turnInches * COUNTS_PER_INCH);
     newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (turnInches * COUNTS_PER_INCH);
     newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() - (int) (turnInches * COUNTS_PER_INCH);

     robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
     robot.rightBackDrive.setTargetPosition(newRightBackTarget);
     robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
     robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

     // Turn On RUN_TO_POSITION
     robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


     // reset the timeout time and start motion.
     runtime.reset();
     robot.leftBackDrive.setPower(Math.abs(speed));
     robot.rightBackDrive.setPower(Math.abs(speed));
     robot.rightFrontDrive.setPower(Math.abs(speed));
     robot.leftFrontDrive.setPower(Math.abs(speed));


     // keep looping while we are still active, and there is time left, and both motors are running.
     // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
     // its target position, the motion will stop.  This is "safer" in the event that the robot will
     // always end the motion as soon as possible.
     // However, if you require that BOTH motors have finished their moves before the robot continues
     // onto the next step, use (isBusy() || isBusy()) in the loop test.
     while (opModeIsActive() &&
     (runtime.seconds() < timeoutS) &&
     (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) {

     // Display it for the driver.
     telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
     telemetry.addData("Path2", "Running at %7d :%7d",
     robot.leftFrontDrive.getCurrentPosition(),
     robot.rightFrontDrive.getCurrentPosition());
     telemetry.update();
     }
     //  sleep(250);   // optional pause after each move
     }
     }
     */





    public void encoderDrive(double speed,
                             double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches,
//                             double liftInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
//        int newLiftInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
//            newLiftInches = robot.liftMotor.getCurrentPosition() + (int)(liftInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
//            robot.liftMotor.setTargetPosition(newLiftInches);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));
            robot.leftFrontDrive.setPower(Math.abs(speed));
//            robot.liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }
            //  sleep(250);   // optional pause after each move
        }
    }


    public void DriveBackwards (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(-Math.abs(speed));
            robot.rightBackDrive.setPower(-Math.abs(speed));
            robot.rightFrontDrive.setPower(-Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(-Math.abs((speed*1)));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
//                        robot.leftFrontDrive.getCurrentPosition(),
//                        robot.rightFrontDrive.getCurrentPosition(),
//                        robot.leftBackDrive.getCurrentPosition(),
//                        robot.rightBackDrive.getCurrentPosition());
//                telemetry.update();
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);



//                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2500);   // optional pause after each move
        }
    }
    public void StrafeRight (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBackDrive.setPower(-Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(-Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(Math.abs((speed*1)));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
//                        robot.leftFrontDrive.getCurrentPosition(),
//                        robot.rightFrontDrive.getCurrentPosition(),
//                        robot.leftBackDrive.getCurrentPosition(),
//                        robot.rightBackDrive.getCurrentPosition());
//                telemetry.update();
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);



//                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }



}








