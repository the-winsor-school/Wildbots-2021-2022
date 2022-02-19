package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "Arm")
@Disabled
public class LiftieArmElla extends LinearOpMode {

    public DcMotor joint;
    public DcMotor spinner;


    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        //drivingLibrary = new DrivingLibrary(this);
        joint = hardwareMap.get(DcMotor.class, "joint");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        //drivingLibrary.setSpeed(1.0);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

        }
    }

    public void jointLevel(int levelNum) {
        joint.setPower(1);
        int waiting = 0;
        // sets the sleep() time to a variable
        // changes wait time depending on level
        if (levelNum == 1) {
            //Bottom Level
            waiting = 500;
        }
        else if (levelNum == 2) {
            //middle level
            waiting = 1000;
        }
        else if (levelNum == 3) {
            //top level
            waiting = 5000;
        }
        sleep(waiting);
        // a stroke of genius TM
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setPower(1);
        sleep(500);
        // pewwwwwww
        spinner.setPower(0);
        joint.setPower(-1);
        // returns to originial position
        sleep(waiting);
        // sleep goes back down
        joint.setPower(0);
    }
}
