package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Disabled
@Autonomous(name = "Spinner Wheels")
public class TankIntakeSpinners extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    public DcMotor rightSpinner;
    public DcMotor leftSpinner;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1.0);
        telemetry.addData("status", "initialized");
        telemetry.update();

        rightSpinner = hardwareMap.get(DcMotor.class, "right spinner");
        leftSpinner = hardwareMap.get(DcMotor.class, "left Spinner");

        waitForStart();

        if (opModeIsActive()) {

            rightSpinner.setPower(1);
            leftSpinner.setPower(1);

        }
    }
}