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

    private DcMotor leftFrontDrive = null;      //wheels
    private DcMotor rightFrontDrive = null;     //wheels
    private DcMotor leftBackDrive = null;       //wheels
    private DcMotor rightBackDrive = null;      //wheels
    private DcMotor rightIntake = null;         //intake
    private DcMotor leftIntake = null;          //intake
    private DcMotor craneMotor = null;          //lifting stones up (vertical extension)

    private Servo clamp = null;             //grabbing stone when it is inside robot
    private CRServo push = null;            //pushing the stone closer to the clamping mechanism
    private CRServo stoneRotator = null;    //once stone is grabbed move outside robot
    private CRServo skystoneGrabber = null; //auto servo to drag skystone
  //  private Servo ramp = null;            //lift ramp (old)
  //  private Servo intakeDrop = null;      //push intake down (old)
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
        rightIntake = hardwareMap.get(DcMotor.class,"right_intake");
        leftIntake = hardwareMap.get(DcMotor.class,"left_intake");
        craneMotor = hardwareMap.get (DcMotor.class, "crane");

        clamp = hardwareMap.servo.get("grabber");
        skystoneGrabber = hardwareMap.crservo.get("skystone");
        push = hardwareMap.crservo.get("push");
        stoneRotator = hardwareMap.crservo.get("stone_rotator");
     //   intakeDrop = hardwareMap.servo.get("drop");
     //   ramp = hardwareMap.servo.get("ramp");
        foundation = hardwareMap.servo.get("foundation");




//Set the Direction for the motors to turn when the robot moves forward//
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        craneMotor.setDirection (DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);




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
        double IntakePower;
        double cranePower;
        double extend = gamepad2.left_stick_y;
        double intakeSpinPower;
        double flipperPower;
    //power variables to be modified and set the motor powers to.

//Drive, turning, and strafe//
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

//driveing formula. calculates power to each wheel based on joystick position. don't touch

        leftFrontPower = Range.clip(drive + turn + strafe, -0.75, 0.75);
        rightFrontPower = Range.clip(drive - turn - strafe, -0.75, 0.75);
        leftBackPower = Range.clip(drive + turn - strafe, -0.75, 0.75);
        rightBackPower = Range.clip(drive - turn + strafe, -0.8, 0.8);
        cranePower = Range.clip(extend,-1.0,1.0);



// SLOW WHEELS
        if (gamepad1.b) {
            leftBackPower = leftBackPower /2;
            rightBackPower = rightBackPower /2;
            leftFrontPower = leftFrontPower /2;
            rightFrontPower = rightFrontPower /2;
        }
        else {
            leftBackPower = leftBackPower + 0;
            rightBackPower = rightBackPower + 0;
            leftFrontPower = leftFrontPower + 0;
            rightFrontPower = rightFrontPower + 0;
        }

        //sprint

        if (gamepad1.a){
            leftBackPower = leftBackPower * 1.25;
            rightBackPower = rightBackPower * 1.25;
            leftFrontPower = leftFrontPower * 1.25;
            rightFrontPower = rightFrontPower * 1.25;
        }
        else {
            leftBackPower = leftBackPower + 0;
            rightBackPower = rightBackPower + 0;
            leftFrontPower = leftFrontPower + 0;
            rightFrontPower = rightFrontPower + 0;
        }
// SLOW EXTENSION
       /*if (gamepad2.y) {
           extendPower = extendPower / 2;
       }
       else {
           extendPower = extendPower + 0;
       }*/


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

        craneMotor.setPower(cranePower);


        //lift motor



        //intake flip



        //intake spin
        if (gamepad1.right_trigger>0.1){
           IntakePower=0.75;
           rightIntake.setPower(IntakePower);
           leftIntake.setPower(-IntakePower);
        }
        else if (gamepad1.left_trigger>0.1){
            IntakePower=-0.75;
            rightIntake.setPower(IntakePower);
            leftIntake.setPower(-IntakePower);
        }
        else {
            IntakePower=0;
            rightIntake.setPower(IntakePower);
            leftIntake.setPower(-IntakePower);
        }

        if(gamepad2.right_bumper){
            clamp.setPosition(180);
        }
        else if (gamepad2.left_bumper){
            clamp.setPosition(0);
        }
        else {
            clamp.setPosition(0);
        }

        if (gamepad2.left_trigger>0.1){
            push.setPower(-2.0);
        }
        else if (gamepad2.right_trigger>0.1){
            push.setPower(1.0);
        }
        else{
            push.setPower(1.0);
        }

        if (gamepad2.dpad_left){
            stoneRotator.setPower(0.5);
        }
        else if (gamepad2.dpad_right) {
            stoneRotator.setPower(-0.5);
        }
        else {
            stoneRotator.setPower(0);
        }
//        if (gamepad1.y){
//            ramp.setPosition(160);
//        }
//
//        else if (gamepad1.x)ramp.setPosition(0);

        if (gamepad1.dpad_up)skystoneGrabber.setPower(0.5);

        else if (gamepad1.dpad_down)skystoneGrabber.setPower(-0.5);

        else skystoneGrabber.setPower(0.0);


        if (gamepad2.dpad_down) {foundation.setPosition(-0.5);}

        else if (gamepad2.dpad_up) {foundation.setPosition(1);}

        else {
            foundation.setPosition(0.7);
        }


        telemetry.addData("status", "loop 2");
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


