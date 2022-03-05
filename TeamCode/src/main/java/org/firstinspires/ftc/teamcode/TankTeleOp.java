package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;


// import org.firstinspires.ftc.libraries.AutonLibrary;
// import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name = "TeleOp")
public class TankTeleOp extends LinearOpMode {

    private TankDrive tankDrive;
    public Servo cappingServo;

    public DcMotor boxWheels;
    //public DcMotor rotini;

    //AnalogInput forceSensitiveResistor;
    public DcMotor duckSpinner;
    public DcMotor LED;
    public final static double capStart = 0.0;
    public static double capPos = 0.5;
    final double capIncrement = 0.001;

    //private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);

        cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        LED= hardwareMap.get(DcMotor.class, "LED");
        boxWheels = hardwareMap.get(DcMotor.class, "boxWheels");
        cappingServo.setPosition(capStart);
        //rotini = hardwareMap.get(DcMotor.class, "rotini");


        double currentForce;
        //forceSensitiveResistor = hardwareMap.get(AnalogInput.class, "Force Sensitive Resistor");
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
            LED.setPower(1);
            currentForce = tankDrive.forceSensitiveResistor.getVoltage();
            //telemetry.addData("rotini", "current height =" + tankDrive.getRotiniHeight());
            //telemetry.addData("rotini", "target height =" + rotiniTarget);
            tankDrive.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
            // use joysticks for Tankalicious teleop
            telemetry.update();

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
            if(gamepad2.dpad_left) {//outake
                boxWheels.setPower(-1);
            }

            if(gamepad2.dpad_right) {//intake
                boxWheels.setPower(1);
            }

            if(gamepad2.dpad_up) {
                boxWheels.setPower(0);
            }

            //Arm
            /*
            if (rotiniTarget > tankDrive.getRotiniHeight()) {
               43553434r5454 tankDrive.rotini.setPower(-1);
                telemetry.addData("status", "raising rotini");
            } else if (rotiniTarget < tankDrive.getRotiniHeight()) {
                tankDrive.rotini.setPower(1);
                telemetry.addData("status", "lowering rotini");
            } else {
                tankDrive.rotini.setPower(0);
            }
            */

            tankDrive.rotini.setPower(-gamepad2.left_stick_y);

            if (gamepad2.y) {
                tankDrive.moveRotiniToAPosition(15);
                boxWheels.setPower(1);
                sleep(500);
                boxWheels.setPower(0);
                tankDrive.moveRotiniToAPosition(0);
            }

            if (gamepad2.b) {
                tankDrive.moveRotiniToAPosition(9);
                boxWheels.setPower(1);
                sleep(500);
                boxWheels.setPower(0);
                tankDrive.moveRotiniToAPosition(0);
            }

            if (gamepad2.a) {
                tankDrive.moveRotiniToAPosition(3);
                boxWheels.setPower(1);
                sleep(500);
                boxWheels.setPower(0);
                tankDrive.moveRotiniToAPosition(0);
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

            //cappingServo.setPosition();


            if (gamepad2.right_stick_y > 0) {
                 capPos = capPos + capIncrement;
                 cappingServo.setPosition(capPos);
                 telemetry.addData("Position: ", capPos);
                // moves capping servo up by an increment of 0.01
            }

            if (gamepad2.right_stick_y < 0) {
                capPos = capPos - capIncrement;
                cappingServo.setPosition(capPos);
                telemetry.addData("Position: ", capPos);
                // moves capping servo down by an increment of 0.01
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