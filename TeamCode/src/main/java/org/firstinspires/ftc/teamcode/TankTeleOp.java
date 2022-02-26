package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.libraries.AutonLibrary;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.teamcode.TankDrive;



@TeleOp(name = "TeleOp")
public class TankTeleOp extends LinearOpMode {

    private TankDrive tankDrive;
    //public Servo cappingServo;

    public DcMotor boxWheels;
    public DcMotor rotini;

    AnalogInput forceSensitiveResistor;
    public DcMotor duckSpinner;


    //private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);

        //cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        boxWheels = hardwareMap.get(DcMotor.class, "boxWheels");
        rotini = hardwareMap.get(DcMotor.class, "rotini");


        double currentForce;
        forceSensitiveResistor = hardwareMap.get(AnalogInput.class, "Force Sensitive Resistor");
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        boolean alreadyPrinted = false;
        int rotiniTarget = 0;
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (!alreadyPrinted) {
                telemetry.addData("status", "OKAY WE'RE IN THE LOOP");
                alreadyPrinted = true;
            }
            currentForce = forceSensitiveResistor.getVoltage();
            //telemetry.addData("rotini", "current height =" + tankDrive.getRotiniHeight());
            //telemetry.addData("rotini", "target height =" + rotiniTarget);
            tankDrive.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
            // use joysticks for Tankalicious teleop
            telemetry.update();


            if (gamepad1.b) {
                tankDrive.brakeStop();
            }

            //Mech Controller

            //Carousel
            if (gamepad2.left_bumper) {//spins carousel
                duckSpinner.setPower(-1);
                // spins duck spinner
            }

            if (gamepad2.right_bumper) {//spins carousel
                duckSpinner.setPower(1);
                // spins duck spinner
            }

            if (gamepad2.x) {//stops carousel
                duckSpinner.setPower(0);
                //stops duck spinning motor
            }

            //Intake Outtake
            if(gamepad2.dpad_left) {//intake
                boxWheels.setPower(-1);
            }

            if(gamepad2.dpad_right) {//outake
                boxWheels.setPower(1);
            }

            if(gamepad2.dpad_up) {
                boxWheels.setPower(0);
            }

            //Arm
            /*
            if (rotiniTarget > tankDrive.getRotiniHeight()) {
                tankDrive.rotini.setPower(-1);
                telemetry.addData("status", "raising rotini");
            } else if (rotiniTarget < tankDrive.getRotiniHeight()) {
                tankDrive.rotini.setPower(1);
                telemetry.addData("status", "lowering rotini");
            } else {
                tankDrive.rotini.setPower(0);
            }
            */

            rotini.setPower(gamepad2.left_stick_y);

            if (gamepad2.y) {
                //Encoders Arm Level 3
            }

            if (gamepad2.b) {
                //Ecoders Arm Level 2
            }

            if (gamepad2.a) {
                //Encoders Arm Level 1
            }

            //:) from Juila Reynolds


/*
            if (gamepad2.dpad_up) {
                // up arrow
                tankDrive.rotini.setPower(0.5);
                sleep(1000);
                tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.rotini.setPower(0);
                // moves arm up
            }
            //MAKES ROTINI GO DOWN
            if (gamepad2.dpad_down) {
                // down arrow
                tankDrive.rotini.setPower(-0.5);
                sleep(1000);
                tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.rotini.setPower(0);
                // moves arm down
            }
            */



            /*
            if (gamepad2.dpad_left) {
                cappingServo.setPosition(45);
                // moves capping servo 45 degrees to the right
            }
            *
            if (gamepad2.dpad_right) {
                cappingServo.setPosition(-45);
                // moves capping servo 45 degrees to the left(?)
            }
            */


            if (currentForce > 0.113 && currentForce < 0.169) {
                telemetry.addData("Box Weight:", "Light");
                telemetry.update();
            } else if (currentForce > 0.189 && currentForce < 0.224) {
                telemetry.addData("Box Weight:", "Medium");
                telemetry.update();
            } else if (currentForce > 0.235 && currentForce < 0.278){
                telemetry.addData("Box Weight:", "Heavy");
                telemetry.update();
            } else {
                telemetry.addData("Force", currentForce);
                telemetry.update();
            }

        }
    }

    public void rotiniBrake () { //turn the motor off
        tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}