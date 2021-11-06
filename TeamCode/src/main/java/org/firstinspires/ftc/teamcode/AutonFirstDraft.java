package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "First Draft of Auton")
public class AutonFirstDraft extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    //Servo spinningArm;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);
        telemetry.addData("status", "initialized");
        telemetry.update();

        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        waitForStart();

        if (opModeIsActive()) {
            //Path 1:
            //detect the level to place the element based on the position of the shipping piece on the barcode
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            //servo arm lifted to spin carousel
            //spinningArm.setPosition(1);
            //spin the carousel
            drivingLibrary.spinToAngle((25 * Math.PI) / 18);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(3000);
            drivingLibrary.brakeStop();
            //spinningArm.setPosition(1); //potentially a different arm to place preloaded object onto a level
            drivingLibrary.spinToAngle((11 * Math.PI) / 36);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(1000);
            drivingLibrary.brakeStop();

            /*
            //Path 2:
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            //spin carousel
            drivingLibrary.spinToAngle((Math.PI * 25) / 18);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(3000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI/4);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI/2);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();

            //Path 3:
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle((Math.PI * 5) / 4);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI / 4);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(1000);
            drivingLibrary.brakeStop();

            //Path 4:
            //may have to turn after scanning
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, 1, 9);
            sleep(300);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle((Math.PI * 23) / 36);
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            */
        }
    }
}