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

        }
    }
}
