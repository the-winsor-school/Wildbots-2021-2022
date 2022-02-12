package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name  = "Liftie Arm Test 2/11/22", group = "Finished")
public class TestingLiftieArm21122 extends LinearOpMode{
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

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

        arm = hardwareMap.get(DcMotor.class, "arm");
        intakeSpinner = hardwareMap.get(DcMotor.class, "spinning");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        left.setDirection(CRServo.Direction.FORWARD);
        right.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //lift the arm
            if(gamepad1.dpad_up) { //move up one level
                arm.setPower(.5);
                sleep(100);
                arm.setPower(0);
            }

            //making the arm go down
            if(gamepad1.dpad_down) {
                arm.setPower(-.5);
                sleep(100);
                arm.setPower(0);
            }

            //adjust the angle of the intake box
            if(gamepad1.a) {
                left.setPower(1);
                right.setPower(1);
            }

            if(gamepad1.x) {
                left.setPower(0);
                right.setPower(0);
            }

            left.setPower(.5);
            right.setPower(.5);

            //intake
            if(gamepad1.b) {
                intakeSpinner.setPower(1);
                sleep(1000);
                intakeSpinner.setPower(0);
            }

            //outtake
            if(gamepad1.y) {
                intakeSpinner.setPower(-1);
                sleep(1000);
                intakeSpinner.setPower(0);
            }

            telemetry.addData("Status", "Running"); //prints to phone
            //telemetry.addData("Brake Mode", drivingLibrary.getMode());

            telemetry.update(); //makes actually print
        }

    }
}
