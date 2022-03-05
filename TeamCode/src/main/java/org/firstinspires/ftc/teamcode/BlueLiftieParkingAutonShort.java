package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.libraries.DrivingLibrary;
// orient with box facing warehouse
@Autonomous(name = "Blue Liftie Parking Auton Short")
public class BlueLiftieParkingAutonShort extends LinearOpMode{

    private DrivingLibrary drivingLibrary;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            drivingLibrary.bevelDrive(-1, 0, 0); //right on the red side with carousel facing warehouse
            //for blue side, x = -1
            sleep(720);

            drivingLibrary.bevelDrive(0, 1, 0);
            sleep(2500);

            drivingLibrary.brakeStop();
        }
    }
}
