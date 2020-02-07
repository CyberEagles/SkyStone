package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
@Disabled

@TeleOp
public class DrivingSusan extends OpMode
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
    private CRServo flipper = null;

    final double INTAKE_SPIN_SPEED = 1.0;

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
        flipper = hardwareMap.crservo.get("flipper");




//Set the Direction for the motors to turn when the robot moves forward//
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection (DcMotor.Direction.REVERSE);
        dropOffMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeFlip.setDirection(DcMotor.Direction.FORWARD);
        extension.setDirection(DcMotor.Direction.REVERSE);


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
        double dropOff = gamepad2.left_stick_y;
        double extend = gamepad2.right_stick_y;
        double intakeSpinPower;
        double flipperPower;


//Drive, turning, and strafe//
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        dropOffPower = Range.clip(dropOff + 0, -0.05, 0.05);
        extendPower = Range.clip (extend + 0, -1, 1);

// SUPER FAST DROP OFF TO RECOVER FROM FALL. USE WITH EXTREME CAUTION.
        // SUPER FAST DROP OFF TO RECOVER FROM FALL. USE WITH EXTREME CAUTION.
        // SUPER FAST DROP OFF TO RECOVER FROM FALL. USE WITH EXTREME CAUTION.
        // SUPER FAST DROP OFF TO RECOVER FROM FALL. USE WITH EXTREME CAUTION.
        if (gamepad2.left_bumper) {
            dropOffPower = dropOffPower *4;
        }
        else if(gamepad2.right_bumper) {
            dropOffPower = dropOffPower /1.2;
        }
        else {
            dropOffPower = dropOffPower + 0;
        }


        //SLOW EXTENSION
        if (gamepad2.dpad_down) {
            extendPower = (extendPower*0.75);
        }
        else {
            extendPower = extendPower;
        }


// SLOW WHEELS
        if (gamepad1.right_bumper) {
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
        dropOffMotor.setPower(-dropOffPower);
        extension.setPower(extendPower);


        //lift motor
        if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0)liftMotor.setPower(-1.0);
        else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)liftMotor.setPower(1.0);
        else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) liftMotor.setPower(0.0);
        else liftMotor.setPower(0.0);


        //intake flip

        if (gamepad2.a && !gamepad2.b) intakeFlip.setPower(-0.6);
        else if (!gamepad2.a && gamepad2.b) intakeFlip.setPower(0.3);
        else if (gamepad2.a && gamepad2.b) intakeFlip.setPower(0);
        else intakeFlip.setPower(0);


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

        //flipper sorter thingy
        if (gamepad2.y && !gamepad2.x) {
            flipperPower = 1.0;
        }

        else if (gamepad2.x && !gamepad2.y) {
            flipperPower = -1.0;
        }

        else {
            flipperPower = 0.0;
        }

        flipper.setPower(flipperPower);

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


