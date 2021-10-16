package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Simple Auton")
public class SimpleAuton extends LinearOpMode {

    private DrivingLibrary drivingLibrary;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            /*
            YOUR CODE HERE
            */
            drivingLibrary.bevelDrive( 0,-1,0);
            // going forward 1
            sleep(1000);
            drivingLibrary.bevelDrive(0,2,9);
            drivingLibrary.spinToAngle(Math.PI/2);
            // spinning 90º (2π=360, π/2=90)

        }
    }
}