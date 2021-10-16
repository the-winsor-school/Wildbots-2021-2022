package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

import java.util.Arrays;


//class name
@TeleOp(name  = "TeleOp Mode", group = "Finished")
public class SimpleTeleop extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;
    int ranOnce = 0;



    public void runOpMode() throws InterruptedException {
        //set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        //max speed ((1 / distance of joy stick from 0)) x the % of motor powers)
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);
        //telemetry prints to the driver phone
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // moves left joystick forward/back/left/right and right joystick spin
            drivingLibrary.bevelDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            // spin 360º when A is pressed
            if (gamepad1.a) {
                drivingLibrary.spinToAngle(2*Math.PI);
            }
        }
    }
}