package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.libraries.DrivingLibrary;
// orient robot with box towards carousel

@Autonomous(name = "Blue Liftie Carousel Auton SU Short")
public class BlueLiftieCarouselAutonSUShort extends LinearOpMode{

    private DrivingLibrary drivingLibrary;
    DcMotor carousel;

    DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        telemetry.addData("status", "initialized");
        telemetry.update();

        arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();

        if (opModeIsActive()) {
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            drivingLibrary.bevelDrive(1, 0, 0); // right
            sleep(400);
            drivingLibrary.bevelDrive(0, 0, 0);
            //go back
            drivingLibrary.bevelDrive(0, 1, 0); //backwards
            sleep(600);
            drivingLibrary.bevelDrive(0, 0, 0);
            // turn 45 degrees
            drivingLibrary.spinToAngle(-Math.PI/3);

            arm.setPower(.5f);
            sleep(700);
            arm.setPower(0);

            // move forward
            drivingLibrary.bevelDrive(0, 1, 0); // this ACTUALLY goes backwards(the right direction) i promise
            sleep(380);
            drivingLibrary.bevelDrive(0, 0, 0); // stop

            // carousel

            carousel.setPower(-.5f);
            sleep(5000);
            carousel.setPower(0);

            drivingLibrary.bevelDrive(0, -1, 0); //backwards
            sleep(30);

            arm.setPower(-.75f);
            sleep(1000);
            arm.setPower(0);

            drivingLibrary.spinToAngle(-Math.PI/2);
            sleep(200);

            //drivingLibrary.bevelDrive(0, -1, 0);
            //sleep(600);

            drivingLibrary.brakeStop();

            drivingLibrary.bevelDrive(1, 0, 0); // right
            sleep(1500);


            drivingLibrary.bevelDrive(0, 1, 0); //forwards
            sleep(200);
            drivingLibrary.brakeStop();
            // move right a little
            /*// 0 degrees
            drivingLibrary.spinToAngle(-.1f);
            // move right a little
            drivingLibrary.bevelDrive(1, 0, 0); // right
            sleep(200);
            // drive forward
            drivingLibrary.bevelDrive(0, -1, 0); //forwards
            sleep(5000);
            drivingLibrary.brakeStop(); */
        }
    }
}