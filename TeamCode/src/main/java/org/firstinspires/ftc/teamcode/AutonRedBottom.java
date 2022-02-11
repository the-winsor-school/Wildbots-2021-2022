package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Disabled
@Autonomous(name = "Auton Red Bottom")
public class AutonRedBottom extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    //Servo spinningArm;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive tankDrive=new TankDrive(this);
        telemetry.addData("status", "initialized");
        telemetry.update();

        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        waitForStart();

        if (opModeIsActive()) {

            tankDrive.drive(-1, -1);
            sleep(1500);
            tankDrive.brakeStop();
            //spin carousel
            tankDrive.spinToAngle(Math.PI - (Math.PI / 4));
            sleep(1000);
            tankDrive.brakeStop();
            tankDrive.drive(-1, -1);
            sleep(3000);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI -(Math.PI/4));
            sleep(1000);
            tankDrive.brakeStop();
            tankDrive.drive(-1 , -1);
            sleep(2000);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI + (Math.PI / 2));
            sleep(1000);
            tankDrive.brakeStop();
            tankDrive.drive(-1, -1);
            sleep(2000);
            tankDrive.brakeStop();

        }
    }
}