package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp


public class DrivingOnlyChassis extends OpMode {
    //Declare motors and variables//
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void init() {
        //Declare variables for phone to recognise//
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
//Set the Direction for the motors to turn when the robot moves forward//
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
        //assuming DOWN and RIGHT are POSITIVE, Y is up/down, X is left/right
        double drive = -gamepad1.left_stick_y; //-gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x*0.4; //gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x; //gamepad1.right_stick_x;


//        double driveTurn = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(-drive + turn - strafe, -1.0, 1.0);
        leftBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightBackPower = Range.clip(-drive + turn + strafe, -1.0, 1.0);

        telemetry.addData("LY",drive);
        telemetry.addData("LX",strafe);
        telemetry.addData("RX",turn);
        telemetry.addData("LF",leftFrontPower);
        telemetry.addData("RF",rightFrontPower);
        telemetry.addData("LB",leftBackPower);
        telemetry.addData("RB",rightBackPower);
        telemetry.update();

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
/*
        if (gamepad1.left_stick_y < 1) {
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            leftFrontDrive.setPower(-leftFrontPower);
            rightBackDrive.setPower(rightFrontPower);
        }
*/
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

//Setting the power of the motor//
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(-leftBackPower);
        rightBackDrive.setPower(-rightBackPower);
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