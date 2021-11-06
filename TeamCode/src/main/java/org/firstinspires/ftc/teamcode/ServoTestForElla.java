package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Servo")
public class ServoTestForElla extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    Servo armServo;
    Servo leftServo;
    Servo rightServo;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        armServo = hardwareMap.get(Servo.class, "arm Servo");
        leftServo = hardwareMap.get(Servo.class, "left Servo");
        rightServo = hardwareMap.get(Servo.class, "right Servo");
        drivingLibrary.setSpeed(1.0);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            armServo.setPosition(0);
            armServo.setPosition(1);
            boxServos(0);
            boxServos(1);
        }
    }

    public void boxServos(int position){
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}
