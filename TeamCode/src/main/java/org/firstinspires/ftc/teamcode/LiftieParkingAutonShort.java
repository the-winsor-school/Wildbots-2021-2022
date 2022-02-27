package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Liftie Parking Auton Short")
public class LiftieParkingAutonShort extends LinearOpMode{

    private DrivingLibrary drivingLibrary;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            drivingLibrary.bevelDrive(0, 1, 0);
            sleep(1500);

            drivingLibrary.bevelDrive(-1, 0, 0);
            sleep(2000);

            drivingLibrary.brakeStop();
            }

}
