package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name="Adding Time to things", group="Pushbot")


public class AddingTimeToThings extends LinearOpMode {
    //Declare motors and variables//
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @Override    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //ContinueAction=1 second

        DriveForward(1);
        ContinueAction();
        ContinueAction();
        ContinueAction();
        Stop();
        ContinueAction();
        ContinueAction();
        SpinRight(1);
        ContinueAction();
        SpinLeft(1);
        ContinueAction();
        Stop();
        ContinueAction();
        Stop();
        StrafeRight(1);
        ContinueAction();
        StrafeLeft(1);
        ContinueAction();
        DriveBackward(1);
        ContinueAction();
        ContinueAction();
        ContinueAction();

        Stop();


    }

    /** this is a long comment.
     * It has multiple lines
     * type the /** for green or /* for grey to start it
     * and to end it type */

    /* grey comment can also be // if only one line
    and you don't need to have the /* or anything at the beginning of each line.
    You can use these to make your code more understandable for others to read
    */

    //like this. but here i've made a little tutorial for you. Follow along :)


    /**so here I made a drive forward time thingy. I believe its called a "method."
    as you can see, it uses the DriveForward from last time, but it has something new.
    the sleep command is the same one that we use in our ContinueAction method, but instead of being for 1 second,
    we get to set how long we want it.
    we do this by using the time variable, just like how we can change the power on our drive.
    only time isn't a decimal percent. Instead, it is in milliseconds. 1 second = 1000ms.
    if we want to, we can change this so it will do sleep(time*1000) to get seconds.
    */

    public void DriveForwardTime(double power, long time) {
        DriveForward(power);
        sleep(time);
    }

    /* Take a look at what I did here, and try using it.
    Once you get the hang of it, try making and using a SpinRightTime method!
    if you want a couple tips, don't be afraid to ask!
    */


    /**
     So. By now you might have a few errors in your code. you can see all of these thanks to the bar on the right -->
     There are a few common errors when getting new files. Starting at the very top, you will probably have an error
     on your very first line of code. PACKAGE. Think of this as where the code is on your computer. mine is
     package org.firstinspires.ftc.teamcode.Vision;
     because my code is located in the ^ teamcode folder, and then in a Vision folder inside that.
     If this has a red underline (error), try changing it. If you don't know where the file is on your computer, start
     with looking in the project side bar on the left. open that, and find this program in the file tree. It will most
     likely be in TeamCode/java/...
     Find it and edit your package line so you don't have an error there.
     Errors are bad, because they won't let you even try to run the code.

     */



    public void DriveForward(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void DriveBackward(double power) {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }

    public void SpinRight(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }

    public void SpinLeft(double power) {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
    }

    public void StrafeRight(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
    }

    public void StrafeLeft(double power) {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }

    public void ContinueAction() {
        sleep(1000);
    }

    public void Stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}


