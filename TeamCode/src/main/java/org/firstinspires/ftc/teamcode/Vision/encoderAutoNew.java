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
public class encoderAutoNew extends LinearOpMode{
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
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();// enables the camera overlay. this will take a couple of seconds

        waitForStart();
        vision.enable();// enables the tracking algorithms. this might also take a little time
//        ParkTime(0.9,7000);
        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback
//try putting the start of every auto here
            leftBackDrive.setPower(1);
            leftFrontDrive.setPower(1);
            rightBackDrive.setPower(1);
            rightFrontDrive.setPower(1);
            DriveForwardTicks(1120);

            ParkTime(1,3000);
            switch (goldPosition){ // for using things in the autonomous program
                case LEFT:
                    telemetry.addLine("This is really right");
                    telemetry.update();
                    break;
                case CENTER:
                    telemetry.addLine("This is really left");
                    telemetry.update();
                    break;
                case RIGHT:
                    telemetry.addLine("going to the center");
                    telemetry.update();
                    break;
                case UNKNOWN:
                    telemetry.addLine("error. going center");
                    telemetry.update();
                    break;
            }
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

