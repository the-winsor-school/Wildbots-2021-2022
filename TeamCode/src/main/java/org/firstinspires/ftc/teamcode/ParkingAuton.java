package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "ParkingAuton")
public class ParkingAuton extends LinearOpMode {

    //private DrivingLibrary drivingLibrary;
    private TankDrive tankDrive;

    public CRServo boxServo;

    //public Servo cappingServo;

    public DcMotor leftIntakeSpinner;
    public DcMotor rightIntakeSpinner;
    public DcMotor frontIntakeSpinner;
    AnalogInput forceSensitiveResistor;

    public int servoPosition;

    public DcMotor duckSpinner;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
//        drivingLibrary = new DrivingLibrary(this);
//        drivingLibrary.setSpeed(1.0);

        tankDrive = new TankDrive(this);

        boxServo = hardwareMap.get(CRServo.class, "boxServo");
        //cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        leftIntakeSpinner = hardwareMap.get(DcMotor.class, "leftIntakeSpinner");
        rightIntakeSpinner = hardwareMap.get(DcMotor.class, "rightIntakeSpinner");
        frontIntakeSpinner = hardwareMap.get(DcMotor.class, "frontIntakeSpinner");



        double currentForce;
        forceSensitiveResistor = hardwareMap.get(AnalogInput.class, "Force Sensitive Resistor");
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        boolean alreadyPrinted = false;
        int rotiniTarget = 0;

        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        waitForStart();

        if (opModeIsActive()) {
            double angle=Math.PI/2;
            //goes forward
            tankDrive.drive(1,1);
            sleep(400);
            //stops
            tankDrive.drive(0,0);
            sleep(500);
            //turns right 90 degrees
            tankDrive.drive(0,1);
            sleep(1000);
            //stop again
            tankDrive.drive(0,0);
            sleep(500);
            //go forward
            tankDrive.drive(1,1);
            sleep(1100);
            tankDrive.brakeStop();


        }
    }
}