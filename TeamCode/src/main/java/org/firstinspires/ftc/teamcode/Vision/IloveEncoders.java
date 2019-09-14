package org.firstinspires.ftc.teamcode.Vision;

import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@Autonomous
@Disabled
public class IloveEncoders extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    int TargetTicks = 0;
    public static final double LEFT_FRONT_POWER       =  -1 ;
    public static final double RIGHT_FRONT_POWER    =  1 ;
    public static final double LEFT_BACK_POWER  = -1 ;
    public static final double RIGHT_BACK_POWER = 1 ;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while(opModeIsActive()){

            leftBackDrive.setTargetPosition(3000);
            leftFrontDrive.setTargetPosition(3000);
            rightBackDrive.setTargetPosition(3000);
            rightFrontDrive.setTargetPosition(3000);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            DriveForwardTicks(22400);
            leftBackDrive.setPower(1);
            leftFrontDrive.setPower(1);
            rightBackDrive.setPower(1);
            rightFrontDrive.setPower(1);

//            DriveForwardTicks(448000000);

            ParkTime(1,3000);

//putting the ending of every auto here
            ParkTime(1,30000);
        }

        vision.shutdown();
    }    double ParkTime (double power, long time) throws InterruptedException {
        Park(power);
        Thread.sleep(time);
        return 0;
    }
//    double LowerDownTime (double power,long time) throws InterruptedException {
//        LowerDown(-power);
//        Thread.sleep(time);
//        return 0;
//    }
//    double RotateTime (double power, long time) throws InterruptedException {
//        Rotation(power);
//        Thread.sleep(time);
//        return 0;
//    }

    double DriveForwardTicks (int TargetTicks) throws InterruptedException {
        DriveForward(TargetTicks);
        return 0;
    }
//    double StrafeRightTime (double power, long time) throws InterruptedException {
//        StrafeRight(power);
//        Thread.sleep(time);
//        return 0;
//    }
//    double MarkerDropTime (double power, long time) throws InterruptedException {
//        DropMarker(power);
//        Thread.sleep(time);
//        return 0;
//    }


    //    public void LowerDown(double power) {
//        liftMotor.setPower(-power);
//    }
//    public void StrafeRight(double power) {
//        leftFrontDrive.setPower(power);
//        leftBackDrive.setPower(-power);
//        rightBackDrive.setPower(power);
//        rightFrontDrive.setPower(-power);
//    }
//    public void Rotation (double power){
//        leftFrontDrive.setPower(power);
//        leftBackDrive.setPower(power);
//        rightFrontDrive.setPower(-power);
//        rightBackDrive.setPower(-power);
//    }
    public void DriveForward (int TargetTicks) {
        leftFrontDrive.setTargetPosition(-TargetTicks);
        leftBackDrive.setTargetPosition(-TargetTicks);
        rightBackDrive.setTargetPosition(TargetTicks);
        rightFrontDrive.setTargetPosition(TargetTicks);
    }
    //1120 = 1 revolution
//    public void DropMarker (double power) {
//        teamMarker.setPosition( -power);
//    }

    public void Park (double power){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}

