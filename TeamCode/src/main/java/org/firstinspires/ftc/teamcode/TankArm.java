package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.enums.Encoders;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "Lasagne")
public class TankArm<encoderValues> extends LinearOpMode {

    //define things here
    private DrivingLibrary drivingLibrary;
    Servo armServo;
    Servo leftServo;
    Servo rightServo;
    public DcMotor rotini;
    private int encoderValues = 0;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        //drivingLibrary = new DrivingLibrary(this);
        rotini = hardwareMap.get(DcMotor.class, "rotini");
        leftServo = hardwareMap.get(Servo.class, "left Servo");
        rightServo = hardwareMap.get(Servo.class, "right Servo");
        //drivingLibrary.setSpeed(1.0);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            boxServos(45);
            sleep(5000);
            boxServos(45);
            sleep(5000);
            //boxServos(0);

            rotini.setPower(-1); // -1 = goes up
            sleep(1800);  //lasagne lifts for 1800
            rotini.setPower(0); //stops rotini
            rotiniBrake(); //brakes rotini
            boxServos(90);
            sleep(5000);
            boxServos(135);

            rotini.setPower(0); //cancels brake
            rotini.setPower(1); // 1 = goes down
            sleep(1800);
            rotini.setPower(0);

        }
    }

    public void boxServos(int position) {
        leftServo.setPosition(position);
        rightServo.setPosition(180 - position);
    }

    public void rotiniBrake() {
        rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}