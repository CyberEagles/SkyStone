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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.HardwarePushbot;

import java.util.List;

/**
 * Tensorflow and Encoder code
 */

@Autonomous(name="Tensorflow Auto", group="Pushbot")

public class TensorflowAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;




    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use our robot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75 ;     // This is < 1.0 if geared UP //0.75 for chassis, x for susan
    static final double     LIFT_GEAR_REDUCTION     = 2;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     NO_WHEEL                = 0.5;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_LIFT_INCH    = (COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION) / (NO_WHEEL * 3.1415);
    static final double     DRIVE_SPEED             = 0.7 ;
    static final double     TURN_SPEED              = 0.5;
    static final double     LIFT_SPEED              = 1;


    @Override
    public void runOpMode() {
        while (!opModeIsActive() && !isStopRequested()) {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
        }
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.intakeFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.dropOffMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.intakeFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.dropOffMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //tensorflow

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
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



        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
//look at the first stone
                if (lookForSkystone()) {
                    //grab it
                    telemetry.addData("Skystone Detected! Position","1");
                    telemetry.update();
                    sleep(5000);
                } else {
                    //drive to next one
                    telemetry.addData("Skystone Not Found! Continue to Position","2");
                    telemetry.update();
                    sleep(5000);
                }

//look again at the next stone
                if (lookForSkystone()) {
                    //grab it
                    telemetry.addData("Skystone Detected! Position","2");
                    telemetry.update();
                    sleep(5000);
                } else {
                    //drive to the next one and grab it
                    telemetry.addData("Skystone Not Found! Continue to Position","3");
                    telemetry.update();
                    sleep(5000);
                }

            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
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
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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

        if (opModeIsActive()) {

            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            runtime.reset();
            robot.leftBackDrive.setPower(-Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(-Math.abs((speed*1)));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            sleep(250);   // optional pause after each move
        }
    }

    public void TurnRightNew (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        if (opModeIsActive()) {

            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            runtime.reset();
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(-Math.abs(speed));
            robot.rightFrontDrive.setPower(-Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(Math.abs((speed*1)));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {
            }

            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftBackDrive.setPower(0);

            sleep(250);   // optional pause after each move
        }
    }

  /**
    public void lift (double speed, double liftInches, double timeoutS) {
        int newLiftTarget;

        if (opModeIsActive()) {
            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int) (liftInches * COUNTS_PER_INCH);

            robot.liftMotor.setTargetPosition(newLiftTarget);

            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newLiftTarget);
                telemetry.addData("Path2", "Running at %7d",
                        robot.liftMotor.getCurrentPosition(),
                        telemetry.update());

            }
        }
    }
*/

    public void Wait ()   {
        sleep(1000);
    }
    public void Wait500 ()   {
        sleep(500);
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;

        if (opModeIsActive()) {

            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));
            robot.leftFrontDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }
        }
    }

/**
    public void nonWheelMotors(double speed,
                               double LiftTarget,
                               double timeoutS) {
        int newLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int)(LiftTarget * COUNTS_PER_LIFT_INCH);

            robot.liftMotor.setTargetPosition(newLiftTarget);

            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.liftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newLiftTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        robot.liftMotor.getCurrentPosition(),
                        telemetry.update());
            }
            //  sleep(250);   // optional pause after each move
        }
    }
*/

    public void DriveBackwards (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        if (opModeIsActive()) {

            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            runtime.reset();
            robot.leftBackDrive.setPower(-Math.abs(speed));
            robot.rightBackDrive.setPower(-Math.abs(speed));
            robot.rightFrontDrive.setPower(-Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(-Math.abs((speed*1)));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

        }
    }
    public void StrafeRight (double speed, double driveInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftFrontTarget;
        int newLiftInches;

        if (opModeIsActive()) {

            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int) (driveInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (driveInches * COUNTS_PER_INCH);

            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            runtime.reset();
            robot.leftBackDrive.setPower(-Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(-Math.abs((speed*1)));
            robot.leftFrontDrive.setPower(Math.abs((speed*1)));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFrontDrive.isBusy() )) { // && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())) {
            }

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            sleep(250);   // optional pause after each move
        }
    }
   /**
    public void Intake (double power) {
        robot.intakeFlip.setPower(power);
    }

    double IntakeTime (double power, long time) throws InterruptedException {
        Intake(power);
        Thread.sleep(time);
        return 0;
    }
    */
}








