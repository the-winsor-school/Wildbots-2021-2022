package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Blue Liftie Warehouse Auton Carousel Long")
public class LiftieParkingAutonCarouselLongBlue extends LinearOpMode{

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

            // strafe right
            drivingLibrary.bevelDrive(1, 0, 0); // right
            sleep(400);
            drivingLibrary.bevelDrive(0, 0, 0);
            //go back
            drivingLibrary.bevelDrive(0, 1, 0); //backwards
            sleep(1500);
            drivingLibrary.bevelDrive(0, 0, 0);
            // turn 45 degrees
            drivingLibrary.spinToAngle(-Math.PI/4);

            // move forward
            drivingLibrary.bevelDrive(0, 1, 0); // this ACTUALLY goes backwards(the right direction) i promise
            sleep(300);
            drivingLibrary.bevelDrive(0, 0, 0); // stop

            // carousel

            carousel.setPower(-1);
            sleep(3500);
            carousel.setPower(0);

            // 0 degrees
            drivingLibrary.spinToAngle(.1f);
            // move right a little
            drivingLibrary.bevelDrive(1, 0, 0); // right
            sleep(200);
            // drive forward
            drivingLibrary.bevelDrive(0, -1, 0); //forwards
            sleep(5000);
            drivingLibrary.brakeStop();
        }
    }
}