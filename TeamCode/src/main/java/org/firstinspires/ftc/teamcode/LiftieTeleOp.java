package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name  = "Liftie TeleOp", group = "Finished")
public class LiftieTeleOp extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

    Rev2mDistanceSensor barcode;
    DcMotor carousel;
    DcMotor arm;
    DcMotor intakeSpinner;
    CRServo left;
    CRServo right;

    public void runOpMode() throws InterruptedException {
        //set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        barcode = hardwareMap.get(Rev2mDistanceSensor.class, "barcode");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intakeSpinner = hardwareMap.get(DcMotor.class, "spinning");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        left.setDirection(CRServo.Direction.FORWARD);
        right.setDirection(CRServo.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //driving
            drivingLibrary.bevelDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            // switching braking modes
            if (gamepad1.b) {
                drivingMode++;
                drivingMode %= DrivingMode.values().length;
                drivingLibrary.setMode(drivingMode);
            }

            //lift the arm
//            if(gamepad2.dpad_up) { //move up one level
//                arm.setPower(1);
//                sleep(500);
//                arm.setPower(0);
//            }

            arm.setPower(gamepad2.left_stick_y * 0.5);

            //making the arm go down
//            if(gamepad2.dpad_down) {
//                arm.setPower(-1);
//                sleep(500);
//                arm.setPower(0);
//            }

           /* //adjust the angle of the intake box
            if(gamepad2.a) {
                left.setPower(1);
                right.setPower(1);
            }

            if(gamepad2.b) {
                left.setPower(0);
                right.setPower(0);
            }

            left.setPower(.5);
            right.setPower(.5); */

            // cr servos
            left.setPower(gamepad2.right_stick_y);
            right.setPower(gamepad2.right_stick_y);

            //intake
            if(gamepad2.x) {
                intakeSpinner.setPower(1);
            }

            //outake
            else if(gamepad2.y) {
                intakeSpinner.setPower(-1);
            }

            else {
                intakeSpinner.setPower(0);
            }

            //get distance
            if(gamepad2.right_bumper) {
                barcode.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance", barcode.getDistance(DistanceUnit.CM));
                telemetry.update();
            }

            //carousel ducks
            if(gamepad2.dpad_left) {
                carousel.setPower(1);
            }

            if(gamepad2.dpad_right) {
                carousel.setPower(-1);
            }

            if(gamepad2.dpad_down) {
                carousel.setPower(0);
            }

            telemetry.addData("Status", "Running"); //prints to phone
            telemetry.addData("Left CR Servo power", left.getPower()); //prints to phone
            telemetry.addData("Right CR Servo power", right.getPower()); //prints to phone
            telemetry.addData("Left CR Servo direction", left.getDirection()); //prints to phone
            telemetry.addData("Right CR Servo direction", right.getDirection()); //prints to phone
            //telemetry.addData("Brake Mode", drivingLibrary.getMode());

            telemetry.update(); //makes actually print
        }
    }
}