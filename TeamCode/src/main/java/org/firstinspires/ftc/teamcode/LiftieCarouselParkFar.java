package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Liftie Auton Park")
public class LiftieCarouselParkFar extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    Rev2mDistanceSensor barcode;
    DcMotor carousel;
    DcMotor arm;
    DcMotor spinning;
    CRServo left;
    CRServo right;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);
        barcode = hardwareMap.get(Rev2mDistanceSensor.class, "barcode");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        spinning = hardwareMap.get(DcMotor.class, "spinning");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        left.setDirection(CRServo.Direction.FORWARD);
        right.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
//            drivingLibrary.bevelDrive(0, 1, 0);
//            sleep(4000);
//            drivingLibrary.brakeStop();
//            carousel.setPower(1);
//            sleep(3500);
//            carousel.setPower(0);
//            drivingLibrary.bevelDrive(0, -1, 0);
//            sleep(7000);
//            drivingLibrary.brakeStop();
            sleep(5000);
            drivingLibrary.bevelDrive(0, 1, 0);
            sleep(3000);
            drivingLibrary.brakeStop();
            carousel.setPower(-1);
            sleep(3500);
            carousel.setPower(0);
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(1000);
            drivingLibrary.bevelDrive(-1, 0, 0);
            sleep(8000);
            drivingLibrary.brakeStop();
        }
    }
}