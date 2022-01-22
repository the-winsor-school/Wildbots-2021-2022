package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;


@TeleOp(name  = "Liftie Drive", group = "Finished")
public class LiftieDrive extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;
    DcMotor lifter;
    DcMotor intake;

    public void runOpMode() throws InterruptedException {
        //set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);
        lifter = hardwareMap.get(DcMotor.class, "lifter");;
        intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //driving
            drivingLibrary.bevelDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if(gamepad1.b) {
                drivingLibrary.brakeStop(); //driving brakestop (instead of float stop)
            }

            if(gamepad2.dpad_up) { //move one increment up
                lifter.setPower(1);
                sleep(500);
                lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if(gamepad2.dpad_down) { //two increments can move from top to bottom
                lifter.setPower(-1);
                sleep(500);
                lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            intake.setPower(gamepad2.right_stick_y); //based on positioning of the joystick, motor will intake or outtake

            telemetry.addData("Status", "Running"); //prints to phone
            telemetry.addData("Brake Mode", drivingLibrary.getMode());

            telemetry.update(); //makes actually print
        }
    }
}
