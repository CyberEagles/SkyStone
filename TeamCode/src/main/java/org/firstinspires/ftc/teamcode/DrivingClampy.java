package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class DrivingClampy extends OpMode
{
    //Declare motors and variables//


    //Motors: 4 wheels, vertical extension, part one for skystone clamp
    //Servos: intake clamp, intake rotate, foundation, skystone clamp (part one is motor)


    private DcMotor leftFrontDrive = null;      //wheels
    private DcMotor rightFrontDrive = null;     //wheels
    private DcMotor leftBackDrive = null;       //wheels
    private DcMotor rightBackDrive = null;      //wheels
    private DcMotor craneMotor = null;          //lifting stones up (vertical extension)
    private DcMotor skystoneGrabber = null;   //motor to drag skystones in auto, Part 1

    private CRServo clamp = null;             //grabbing stones
    private CRServo claw = null;              //skystone Grabber pinch servo, Part 2
    private CRServo stoneRotator = null;    //once stone is grabbed move outside robot
    private Servo foundation = null;        //grab foundation


    final double INTAKE_SPIN_SPEED = 1.0;

    @Override
    public void init() {
        //Declare variables for phone to recognise//

        //names on the config

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        craneMotor = hardwareMap.get (DcMotor.class, "crane");
        skystoneGrabber = hardwareMap.get(DcMotor.class,"skystone");

        clamp = hardwareMap.crservo.get("clamp");
        stoneRotator = hardwareMap.crservo.get("stone_rotator");
        foundation = hardwareMap.servo.get("foundation");
        claw = hardwareMap.crservo.get("claw");

        //Reset the Encoder
        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skystoneGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the modes for each motor
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        skystoneGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



//Set the Direction for the motors to turn when the robot moves forward//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        craneMotor.setDirection (DcMotor.Direction.FORWARD);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//Tells drivers that robot is ready//
//        telemetry.addData("status", "Initialized");
    }

    @Override
    public void start() {
//        telemetry.addData("status", "start");
    }


    double ExtensionEncoderCounts = 1440;
    double ExtensionPower = 1;
    int ExtensionInches = 4;
    int ExtensionTarget = 300;
    double ExtensionModeToggleVariable = 2;


    //Set variables//
    @Override
    public void loop() {
//        telemetry.addData("status", "loop 1");
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        //power variables to be modified and set the motor powers to.

//Drive, turning, and strafe//
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


//driving formula. calculates power to each wheel based on joystick position. don't touch

        leftFrontPower = Range.clip(drive + turn + strafe, -1, 1);
        rightFrontPower = Range.clip(drive - turn - strafe, -1, 1);
        leftBackPower = Range.clip(drive + turn - strafe, -1, 1);
        rightBackPower = Range.clip(drive - turn + strafe, -1, 1);


// SLOW WHEELS
        if (gamepad1.b) {
            leftBackPower = leftBackPower / 2;
            rightBackPower = rightBackPower / 2;
            leftFrontPower = leftFrontPower / 2;
            rightFrontPower = rightFrontPower / 2;
        } else {
            leftBackPower = leftBackPower + 0;
            rightBackPower = rightBackPower + 0;
            leftFrontPower = leftFrontPower + 0;
            rightFrontPower = rightFrontPower + 0;
        }

        //sprint

//        if (gamepad1.a) {
//            leftBackPower = leftBackPower * 1.25;
//            rightBackPower = rightBackPower * 1.25;
//            leftFrontPower = leftFrontPower * 1.25;
//            rightFrontPower = rightFrontPower * 1.25;
//        } else {
//            leftBackPower = leftBackPower + 0;
//            rightBackPower = rightBackPower + 0;
//            leftFrontPower = leftFrontPower + 0;
//            rightFrontPower = rightFrontPower + 0;
//        }


        // not locking the wheels while turning
        if (gamepad1.right_stick_x >= 0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = 0.2;
            rightBackPower = 0.2;
        } else if (gamepad1.right_stick_x <= -0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = 0.2;
            leftBackPower = 0.2;
        } else if (gamepad1.right_stick_x >= 0.1 && gamepad1.left_stick_y <= -0.1) {
            leftFrontPower = -0.3;
            leftBackPower = -0.3;
        } else if (gamepad1.right_stick_x <= -0.1 && gamepad1.left_stick_y <= -0.1) {
            rightFrontPower = -0.3;
            rightBackPower = -0.3;
        } else {
            rightFrontPower = rightFrontPower;
            rightBackPower = rightBackPower;
            leftFrontPower = leftFrontPower;
            leftBackPower = leftBackPower;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);



        if (gamepad2.right_trigger > 0.5) {
            clamp.setPower(1);
        }
        else if (gamepad2.left_trigger > 0.5){
            clamp.setPower(-1);
        }
        else {
            clamp.setPower(0);
        }



        if (gamepad2.x) {
            stoneRotator.setPower(1);
        } else if (gamepad2.b) {
            stoneRotator.setPower(-1);
        }
        else {stoneRotator.setPower(0);}


        if (gamepad1.dpad_down) skystoneGrabber.setPower(0.5);

        else if (gamepad1.dpad_up) skystoneGrabber.setPower(-0.5);

        else skystoneGrabber.setPower(0);




        if (gamepad1.x) {
            claw.setPower(1);
        }
        else if (gamepad1.y) {
            claw.setPower(-1);
        }
        else claw.setPower(0);

//



        if (gamepad2.right_stick_y > 0.5) {
            foundation.setPosition(-0.5);
        } else if (gamepad2.right_stick_y < -0.5) {
            foundation.setPosition(1);
        } else {
            foundation.setPosition(0.7);
        }

/**Gamepad 2 Mode switch and extension movement
 */

        if (gamepad2.a) {
            ExtensionPower = ExtensionPower*0.5;
        } else {ExtensionPower = 1;
        }

//  DISABLED DUE TO LACK OF TESTING AND TARGET POSITION MOVING WAY TOO MUCH WAY TOO FAST

//        if (gamepad2.dpad_left) {
//            ExtensionModeToggleVariable *= -1;
//        }

/**
        if (ExtensionModeToggleVariable < 0) {

            telemetry.addData("Extension Mode", "TARGET UP/DOWN");
            telemetry.addData("Move up/down with", "RIGHT");
            telemetry.addData("Extension variable", ExtensionModeToggleVariable);
            telemetry.update();

            if (gamepad2.dpad_down && !was_dpad_down) {
                ExtensionTarget -= (ExtensionTarget*ExtensionInches);
                telemetry.addData("Extension target", ExtensionTarget);
                telemetry.update();
            }

            if (gamepad2.dpad_up && !was_dpad_up) {
                ExtensionTarget += (ExtensionTarget*ExtensionInches);
                telemetry.addData("Extension target", ExtensionTarget);
                telemetry.update();
            }


            craneMotor.setTargetPosition(ExtensionTarget);
            craneMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad2.dpad_right) {
                craneMotor.setPower(ExtensionPower);
                telemetry.addData("Moving in target mode towards",ExtensionTarget);
                telemetry.update();
            }

            if (!gamepad2.dpad_right) {
                craneMotor.setPower(0);
            }

        }
*/


        if (ExtensionModeToggleVariable > 0) {
            craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Extension Mode", "POWER UP/DOWN");
            telemetry.addData("Move up/down with", "UP/DOWN");
            telemetry.addData("Extension variable", ExtensionModeToggleVariable);

            telemetry.update();





            if (gamepad2.dpad_up) {
                craneMotor.setPower(ExtensionPower);
                telemetry.addData("Moving...","in power mode. POSITIVE");
                telemetry.update();
            }

            else if (gamepad2.dpad_down) {
                craneMotor.setPower(-ExtensionPower*0.5);
                telemetry.addData("Moving...","in power mode. NEGATIVE");
                telemetry.update();
            }

            else {
                craneMotor.setPower(0.1);
            }
            craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

    }

    //Stop the robot//
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        stoneRotator.setPower(0);
        craneMotor.setPower(0);
        skystoneGrabber.setPower(0);
    }
}


