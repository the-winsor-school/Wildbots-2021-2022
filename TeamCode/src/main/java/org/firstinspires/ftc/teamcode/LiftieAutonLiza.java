package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Liftie Auton From Liza")
@Disabled
public class LiftieAutonLiza extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    Rev2mDistanceSensor barcode;
    DcMotor carousel;
    DcMotor arm;
    DcMotor spinning;
    Servo left;
    Servo right;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);
        barcode = hardwareMap.get(Rev2mDistanceSensor.class, "barcode");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        spinning = hardwareMap.get(DcMotor.class, "spinning");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // strafing to each spot on the barcode
            drivingLibrary.bevelDrive(1, 0, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            int randomized = 0;
            if (barcode.getDistance(DistanceUnit.CM) < 800) {
                randomized = 3;
            }
            drivingLibrary.bevelDrive(1, 0, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            if (barcode.getDistance(DistanceUnit.CM) < 800) {
                randomized = 2;
            }
            drivingLibrary.bevelDrive(1, 0, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            if (barcode.getDistance(DistanceUnit.CM) < 800) {
                randomized = 1;
            }
            drivingLibrary.bevelDrive(1, 0, 0);
            sleep(1500);
            carousel.setPower(1);
            sleep(3500);
            carousel.setPower(0);
            drivingLibrary.bevelDrive(1, 0.5f, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            arm.setPower(1);
            switch (randomized) {
                case 1:
                    sleep(500);
                    break;
                case 2:
                    sleep(1000);
                    break;
                case 3:
                    sleep(1500);
                    break;
            }
            left.setPosition(1);
            right.setPosition(-1);
            sleep(500);
            spinning.setPower(1);
            sleep(1000);
            spinning.setPower(0);
            drivingLibrary.spinToAngle(Math.PI / 2);
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(5000);
            drivingLibrary.brakeStop();
        }
    }
}