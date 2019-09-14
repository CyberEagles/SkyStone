package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@Autonomous
@Disabled
public class DepotAutoV1 extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo teamMarker = null;
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "Ac4K6BD/////AAABmQmEfMSCD0j8hOxBFYTS/CQj/pGySibYpkLudGb7MN12FPBJ3Om18kKjOQTFwk8o9C3FEY0LIcBYqcPMMB35sfLHF2uwiF/9ElfONAFain0CysTHKvL/mOpSZZxOqevoexo9iNlxftfciARbiruvu5kYGZroBCho5R6WHzHjbGfEAGWjIsckDKRvQQkKw5p5N21GqH5ium/tN/TO0asmGwz0RTb4Djt+P8FSynFSJnmC3gsq97fUipK802hR2A4SvzgJKEmrgoNnOxKGx7E/AkBKGY/mIrTy7/Trhq/mnK6STtm4Vl9GBfnsOj3KAexo7JdfFNCPEuKrMXs7wHfvnEYRS4Lwwf/kwSISAQvIN+8N";
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        teamMarker = hardwareMap.servo.get("team_marker");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();// enables the camera overlay. this will take a couple of seconds

        waitForStart();
        vision.enable();// enables the tracking algorithms. this might also take a little time
        ParkTime(0.9,7000);
        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback
// put the start of every auto here
            teamMarker.setPosition(0.0);
            LowerDownTime(-0.9,2725);
            LowerDown(0);
            ParkTime(0.9,100);
            DriveForwardTime(0.9,75);
            DriveForward(0);
            ParkTime(0.9,100);
            StrafeRightTime(-0.9,150);
            StrafeRight(0);
            ParkTime(0.9,100);
            RotateTime(-0.9,500);
            Rotation(0);
            ParkTime(0.9,100);
            DriveForwardTime(-0.9,300);
            DriveForward(0);
            ParkTime(0.9,100);
            RotateTime(-0.9,200);
            Rotation(0);
            ParkTime(0.9,100);

            switch (goldPosition){ // for using things in the autonomous program
                case LEFT:
                    telemetry.addLine("This is really right");
                    telemetry.update();
                    RotateTime(-0.9,425);
                    Rotation(0);
                    ParkTime(0.9,100);
                    DriveForwardTime(-0.9,400);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    DriveForwardTime(0.9,500);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    RotateTime(0.9,450);
                    Rotation(0);
                    ParkTime(0.9,100);

                    break;
                case CENTER:
                    telemetry.addLine("This is really left");
                    telemetry.update();
                    RotateTime(0.9,425);
                    Rotation(0);
                    ParkTime(0.9,100);
                    DriveForwardTime(-0.9,400);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    DriveForwardTime(0.9,500);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    RotateTime(-0.9,450);
                    Rotation(0);
                    ParkTime(0.9,100);
                    break;
                case RIGHT:
                    telemetry.addLine("going to the center");
                    telemetry.update();
                    DriveForwardTime(-0.9,200);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    DriveForwardTime(0.9,150);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    RotateTime(0.9,70);
                    Rotation(0);
                    ParkTime(0.9,100);
                    break;
                case UNKNOWN:
                    telemetry.addLine("error. going center");
                    telemetry.update();
                    DriveForwardTime(-0.9,200);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    DriveForwardTime(0.9,150);
                    DriveForward(0);
                    ParkTime(0.9,100);
                    RotateTime(0.9,70);
                    Rotation(0);
                    ParkTime(0.9,100);
                    break;
            }
//putting the ending of every auto here
            RotateTime(0.9,700);
            Rotation(0);
            ParkTime(0.9,100);
            DriveForwardTime(-0.9,900);
            DriveForward(0);
            ParkTime(0.9,100);
            RotateTime(-0.9,1000);
            Rotation(0);
            DriveForwardTime(-0.9,900);
            DriveForward(0);
            ParkTime(0.9,100);
            RotateTime(-0.9,125);
            Rotation(0);
            ParkTime(0.9,100);
            DriveForwardTime(-0.9,700);
            DriveForward(0);
            ParkTime(0.9,100);
            RotateTime(0.9,800);
            Rotation(0);
            ParkTime(0.9,300);
            teamMarker.setPosition(0.6);
            ParkTime(0.9,300);
            RotateTime(0.9,700);
            Rotation(0);
            ParkTime(0.9,100);
            DriveForwardTime(-0.9,700);
            DriveForward(0);
            ParkTime(0.9,100);
            DriveForwardTime(-0.9,1600);
            ParkTime(0.9,30000);
        }

        vision.shutdown();
    }    double ParkTime (double power, long time) throws InterruptedException {
        Park(power);
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
    public void StrafeRight(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
    }
    public void Rotation (double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }
    public void DriveForward (double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
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
}

