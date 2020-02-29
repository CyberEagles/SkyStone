package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class TwoStoneGrabRed extends LinearOpMode {
    OdometerHardware robot = new OdometerHardware(this);

    @Override

    public void runOpMode() {
        robot.initDriveHardwareMap();
        waitForStart();

        robot.goToPosition(-10, -24, 0.6, 0, 2, 5, robot.BACKWARD);
        robot.skystoneGrabber.setPower(0.5);
        robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(500);
        robot.skystoneGrabber.setPower(0);

        robot.turn(0.5, -80, 3, 5);
        robot.skystoneGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
