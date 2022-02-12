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


    public CRServo boxServo;

    //public Servo cappingServo;

    public DcMotor leftIntakeSpinner;
    public DcMotor rightIntakeSpinner;
    public DcMotor frontIntakeSpinner;
    AnalogInput forceSensitiveResistor;

    public DcMotor duckSpinner;


    //private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
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
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (!alreadyPrinted) {
                telemetry.addData("status", "OKAY WE'RE IN THE LOOP");
                alreadyPrinted = true;
            }
            currentForce = forceSensitiveResistor.getVoltage();
            //telemetry.addData("rotini", "current height =" + tankDrive.getRotiniHeight());
            //telemetry.addData("rotini", "target height =" + rotiniTarget);
            tankDrive.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            // use joysticks for Tankalicious teleop
            telemetry.update();

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

            //LOSING MY MIND HERES WHAT MAKES ROTINI GO UP

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

            if (gamepad2.left_bumper) {//spins carousel
                // letter a
                duckSpinner.setPower(-1);
                sleep(2000); //CHANGE for amount of time to spin duck off
                duckSpinner.setPower(0);
                // spins duck spinner
            }

            if (gamepad2.right_bumper) {//spins carousel
                // letter a
                duckSpinner.setPower(1);
                sleep(2000); //CHANGE for amount of time to spin duck off
                duckSpinner.setPower(0);
                // spins duck spinner
            }

            /*
            if (gamepad2.dpad_up) { //moves box servos up
                boxServo.setPosition(0);
                // moves one servo in one direction and the other in the other direction
            }

            if (gamepad2.dpad_down) { //moves box down
                boxServo.setPosition(90);
                // outtake
            }
            */

            boxServo.setPower(gamepad2.right_stick_x);

            if (gamepad2.b) {//spins intake wheels
                leftIntakeSpinner.setPower(1);
                rightIntakeSpinner.setPower(-1);
                frontIntakeSpinner.setPower(-1);
                sleep(1000);
                leftIntakeSpinner.setPower(0);
                rightIntakeSpinner.setPower(0);
                frontIntakeSpinner.setPower(0);
            }
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