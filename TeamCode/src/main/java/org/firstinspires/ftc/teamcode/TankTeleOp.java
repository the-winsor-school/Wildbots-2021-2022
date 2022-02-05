package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.AutonLibrary;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name = "TeleOp")
public class TankTeleOp extends LinearOpMode {

    private TankDrive tankDrive;


    public Servo boxServo;

    //public Servo cappingServo;

    public DcMotor leftIntakeSpinner;
    public DcMotor rightIntakeSpinner;

    public DcMotor duckSpinner;


    //private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);

        boxServo = hardwareMap.get(Servo.class, "boxServo");
        //cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        leftIntakeSpinner = hardwareMap.get(DcMotor.class, "leftIntakeSpinner");
        rightIntakeSpinner = hardwareMap.get(DcMotor.class, "rightIntakeSpinner");

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
            //telemetry.addData("rotini", "current height =" + tankDrive.getRotiniHeight());
            //telemetry.addData("rotini", "target height =" + rotiniTarget);
            tankDrive.Drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
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

            if (gamepad2.dpad_up) {
                // up arrow
                tankDrive.rotini.setPower(1);
                sleep(500);
                tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.rotini.setPower(0);
                // moves arm up
            }

            if (gamepad2.dpad_down) {
                // down arrow
                tankDrive.rotini.setPower(-1);
                sleep(500);
                tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.rotini.setPower(0);
                // moves arm down
            }
            if(!gamepad2.dpad_up && !gamepad2.dpad_down){
                tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            if (gamepad2.x) {//spins carousel
                // letter a
                duckSpinner.setPower(1);
                sleep(100); //CHANGE for amount of time to spin duck off
                // spins duck spinner
            }

            if (gamepad2.y) { //moves box servos up
                boxServo.setPosition(0);
                // moves one servo in one direction and the other in the other direction
            }

            if (gamepad2.a) { //moves box down
                boxServo.setPosition(90);
                // outtake
            }

            if (gamepad2.b) {//spins intake wheels
                leftIntakeSpinner.setPower(1);
                rightIntakeSpinner.setPower(-1);
                sleep(1000);
                leftIntakeSpinner.setPower(0);
                rightIntakeSpinner.setPower(0);
            }

        }
    }

    public void rotiniBrake () { //turn the motor off
        tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}