package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "ARB")
public class AutonRedBottom extends LinearOpMode {

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

            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(1500);
            drivingLibrary.brakeStop();
            //spin carousel
            drivingLibrary.spinToAngle(Math.PI - (Math.PI / 4));
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(3000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI -(Math.PI/4));
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI + (Math.PI / 2));
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();

        }
    }
}