package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.enums.Encoders;


import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Encodeers ")
public class EncodersTest extends LinearOpMode {

    public DcMotor testMotor;
    private int encoderValues;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {

        testMotor = hardwareMap.get(DcMotor.class, "test motor");

        telemetry.addData("status", "initialized");
        telemetry.update();

        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        waitForStart();

        if (opModeIsActive()) {


        }
    }
}