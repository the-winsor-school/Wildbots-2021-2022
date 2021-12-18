package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;


@TeleOp(name  = "Liftie Drive", group = "Finished")
public class LiftieDrive extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

    public void runOpMode() throws InterruptedException {
        //set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //driving
            drivingLibrary.bevelDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

//            // switching braking modes
//            if (gamepad1.b) {
//                drivingMode = drivingMode == 1 ?  0 : 1;
//                drivingLibrary.setMode(drivingMode);
//          }

            telemetry.addData("Status", "Running"); //prints to phone
            telemetry.addData("Brake Mode", drivingLibrary.getMode());

            telemetry.update(); //makes actually print
        }
    }
}
