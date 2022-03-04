package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Blue Liftie Warehouse Auton Carousel Short")
public class LiftieParkingAutonCarouselShortBlue extends LinearOpMode{

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
            drivingLibrary.bevelDrive(0.6f, 0, 0);
            sleep(400);

            drivingLibrary.bevelDrive(0, 1, 0);
            sleep(750);

            carousel.setPower(-1);
            sleep(3500);
            carousel.setPower(0);

            drivingLibrary.bevelDrive(1, 0, 0);
            sleep(350);

            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(5000);

            drivingLibrary.brakeStop();
        }
    }
}