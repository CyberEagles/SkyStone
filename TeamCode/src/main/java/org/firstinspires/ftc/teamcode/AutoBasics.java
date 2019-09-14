/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto testing", group = "Concept")
@Disabled
public class AutoBasics extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo teamMarker = null;
    private DcMotor extension = null;
    private DcMotor intakeFlip = null;

    public void main() throws InterruptedException {
        //initialize motors
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        teamMarker = hardwareMap.servo.get("team_marker");
        //    leftFrontDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //    rightFrontDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //    leftBackDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //    rightBackDrive.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // Most robots need the motor on one side to be reversed to drive forward

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        teamMarker = hardwareMap.servo.get("team_marker");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        extension = hardwareMap.get(DcMotor.class, "extension");
        intakeFlip = hardwareMap.get(DcMotor.class, "intake_flip");

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                        //land here
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            StrafeRightTime(-1,300);
                            StrafeRight(0);
                            ParkTime(1,100);
                            DriveForwardTime(1,300);
                            DriveForward(0);
                            ParkTime(1,30000);

                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            ParkTime(1,30000);
                          } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            ParkTime(1,30000);
                          }
                        }
                        //rest of auto
                      }
                      telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    double ParkTime (double power, long time) throws InterruptedException {
        Park(power);
        Thread.sleep(time);
        return 0;
    }
    double ExtendTime (double power, long time) throws InterruptedException {
        Extened(power);
        Thread.sleep(time);
        return 0;
    }
    double LowerDownTime (double power,long time) throws InterruptedException {
        LowerDown(-power);
        Thread.sleep(time);
        return 0;
    }
    double RotateTime (double power, long time) throws InterruptedException {
        Rotation(power);
        Thread.sleep(time);
        return 0;
    }
    double IntakeTime (double power, long time) throws InterruptedException {
        Intake(power);
        Thread.sleep(time);
        return 0;
    }

    double DriveForwardTime(double power, long time) throws InterruptedException {
        DriveForward(power);
        Thread.sleep(time);
        return 0;
    }
    double StrafeRightTime (double power, long time) throws InterruptedException {
        StrafeRight(power);
        Thread.sleep(time);
        return 0;
    }
    double MarkerDropTime (double power, long time) throws InterruptedException {
        DropMarker(power);
        Thread.sleep(time);
        return 0;
    }


    public void LowerDown(double power) {
        liftMotor.setPower(-power);
    }
    public void DriveForward(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
    }
    public void Rotation (double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    public void StrafeRight (double power) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
    }
    public void DropMarker (double power) {
        teamMarker.setPosition( -power);
    }


    public void Park (double power){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void Extened (double power) {
        extension.setPower(power);
    }
    public void Intake (double power) {
        intakeFlip.setPower(power);
    }

    @Autonomous
    @Disabled
    public static class encoders extends OpMode
    {
        //Declare motors and variables//

        private DcMotor leftFrontDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightBackDrive = null;
        private DcMotor liftMotor = null;
        private DcMotor dropOffMotor = null;
        private DcMotor intakeFlip = null;
        private DcMotor extension = null;
        private CRServo intakespin = null;

        final double INTAKE_SPIN_SPEED = 1.0;
        private int Encoder_value = 0;

        @Override
        public void init() {
            //Declare variables for phone to recognise//
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
            liftMotor = hardwareMap.get (DcMotor.class, "lift_motor");
            dropOffMotor = hardwareMap.get(DcMotor.class, "drop_off");
            intakeFlip = hardwareMap.get(DcMotor.class, "intake_flip");
            extension = hardwareMap.get(DcMotor.class, "extension");
            intakespin = hardwareMap.crservo.get("intake");




    //Set the Direction for the motors to turn when the robot moves forward//
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            liftMotor.setDirection (DcMotor.Direction.REVERSE);
            dropOffMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeFlip.setDirection(DcMotor.Direction.FORWARD);
            extension.setDirection(DcMotor.Direction.FORWARD);

            extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




    //Tells drivers that robot is ready//
            telemetry.addData("status", "Initialized");
        }

        @Override
        public void start() {
            telemetry.addData("status", "start");
        }

        //Set variables//
        @Override
        public void loop() {
            telemetry.addData("status", "loop 1");

            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;
            double extendPower;
            double dropOffPower;
            double dropOff = gamepad2.right_stick_y;
            double extend = gamepad2.left_stick_y;
            double intakeSpinPower;


    //Drive, turning, and strafe//
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;




            leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
            dropOffPower = Range.clip(dropOff + 0, -0.75, 0.25);
    //        extendPower = Range.clip (extend + 0, -1.0, 1.0);

        if (gamepad2.left_stick_y > 0) {
            Encoder_value = Encoder_value +1;
        }
        else if (gamepad2.left_stick_y < 0) {
            Encoder_value = Encoder_value - 1;
        }

        extension.setTargetPosition(Encoder_value);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // not locking the wheels while turning
            if (gamepad1.right_stick_x >= 0.1 && gamepad1.left_stick_y <= -0.1) {
                rightFrontPower = 0.2;
                rightBackPower = 0.2;
            }

            else if(gamepad1.right_stick_x <=-0.1 && gamepad1.left_stick_y <= -0.1) {
                leftFrontPower = 0.2;
                leftBackPower = 0.2;
            }
            else if(gamepad1.right_stick_x >=0.1 && gamepad1.left_stick_y <= -0.1) {
                leftFrontPower = -0.3;
                leftBackPower = -0.3;
            }
            else if(gamepad1.right_stick_x <= -0.1 && gamepad1.left_stick_y <= -0.1) {
                rightFrontPower = -0.3;
                rightBackPower = -0.3;
            }

            else {
                rightFrontPower = rightFrontPower;
                rightBackPower = rightBackPower;
                leftFrontPower = leftFrontPower;
                leftBackPower = leftBackPower;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            dropOffMotor.setPower(-dropOffPower);
    //        extension.setPower(extendPower);
            extension.setPower(0.2);

            //lift motor
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0)liftMotor.setPower(-1.0);
            else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)liftMotor.setPower(1.0);
            else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) liftMotor.setPower(0.0);
            else liftMotor.setPower(0.0);
            /*
            if (gamepad1.right_trigger > 0)liftMotor.setPower(-1.0);
            else liftMotor.setPower (0.0);
            if (gamepad1.left_trigger > 0)liftMotor.setPower(1.0);
            else liftMotor.setPower(0.0);
    */

            //intake flip

            if (gamepad2.a && !gamepad2.b) intakeFlip.setPower(-0.8);
            else if (!gamepad2.a && gamepad2.b) intakeFlip.setPower(0.3);
            else if (gamepad2.a && gamepad2.b) intakeFlip.setPower(0);
            else intakeFlip.setPower(0);


            /*
            if (gamepad2.a) {
                intakeFlip.setPower(-1.0);
            }
            else {
                intakeFlip.setPower(0);
            }

            if (gamepad2.b) {
                intakeFlip.setPower(0.5);
            }
            else {
                intakeFlip.setPower(0);
            }
    */

            //intake spin
            if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {
                intakeSpinPower = 1.0;
            }

            else if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
                intakeSpinPower = -1.0;
            }

            else if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
                intakeSpinPower = 0;
            }

            else {
                intakeSpinPower = 0.0;
            }

            intakespin.setPower(intakeSpinPower);
            telemetry.addData("Encoder value", Encoder_value);
            telemetry.addData("status", "loop 2");
            telemetry.update();
        }

        //Stop the robot//
        @Override
        public void stop() {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

        }
    }
}