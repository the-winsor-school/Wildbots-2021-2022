package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "DucksAuton")
public class DucksAuton extends LinearOpMode {

    //private DrivingLibrary drivingLibrary;
    private TankDrive tankDrive;

    //public CRServo boxServo;

    //public Servo cappingServo;

    public DcMotor leftIntakeSpinner;
    public DcMotor rightIntakeSpinner;
    public DcMotor frontIntakeSpinner;
    AnalogInput forceSensitiveResistor;
    private Orientation angles;
    private BNO055IMU imu; //gyroscope in rev hub

    public int servoPosition;

    public DcMotor duckSpinner;

    //initializing

    public void turn90Right() {
        TankDrive tankDrive= new TankDrive(this);
        tankDrive.drive(0.5,-0.5);
        sleep(1500);
        tankDrive.brakeStop();
    }
    public void turn90Left() {
        TankDrive tankDrive= new TankDrive(this);
        tankDrive.drive(-0.5,0.5);
        sleep(1500);
        tankDrive.brakeStop();
    }
    public double getIMUAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }
    public void spinToAngle(double angle) {
        double goalAngle = getIMUAngle() + angle;
        while (Math.abs(angle - getIMUAngle()) > .15) {
            if (angle > 0) {
                tankDrive.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                tankDrive.left.setPower(0.25f);
            } else {
                tankDrive.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                tankDrive.right.setPower(0.25f);
            }
            telemetry.update();
        }
        tankDrive.brakeStop();
        //targetAngle = getIMUAngle();
    }
    @Override
    public void runOpMode() throws InterruptedException {
//        drivingLibrary = new DrivingLibrary(this);
//        drivingLibrary.setSpeed(1.0);

        TankDrive tankDrive= new TankDrive(this);
        //boxServo = hardwareMap.get(CRServo.class, "boxServo");
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
            //camera
            tankDrive.drive(1,1);
            sleep(300);
            tankDrive.brakeStop();
            //turn right
            spinToAngle(Math.PI);
            tankDrive.drive(1,1);
            sleep(450);
            tankDrive.brakeStop();
            turn90Left();
            tankDrive.drive(1,1);
            sleep(700);
            tankDrive.brakeStop();

            sleep(700);
            //outake
            turn90Right();
            tankDrive.drive(-1,-1);
            sleep(2000);
            tankDrive.brakeStop();
            turn90Left();
            tankDrive.drive(1,1);
            sleep(500);
            tankDrive.brakeStop();
            sleep(500);
            duckSpinner.setPower(1);
            sleep(1000);
            //spin ducks
            tankDrive.drive(1,1);
            sleep(500);
            tankDrive.brakeStop();

        }
    }
}