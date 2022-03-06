package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.libraries.DrivingLibrary;
// orient robot with box towards carousel
@Autonomous(name = "Red Liftie Carousel Auton SU Short")
public class RedLiftieCarouselAutonSUShort extends LinearOpMode{

    private DrivingLibrary drivingLibrary;
    DcMotor carousel;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            drivingLibrary.bevelDrive(-0.6f, 0, 0); // left
            sleep(600);

            drivingLibrary.bevelDrive(0, .7f, 0); // back
            sleep(1500);
            drivingLibrary.brakeStop();

            drivingLibrary.bevelDrive(.5f, 0, 0); // right
            sleep(100);
            drivingLibrary.brakeStop();

            carousel.setPower(.5f);
            sleep(5000);
            carousel.setPower(0);

            drivingLibrary.bevelDrive(-1, 0, 0);
            sleep(1000);

            drivingLibrary.brakeStop();
        }
    }
}