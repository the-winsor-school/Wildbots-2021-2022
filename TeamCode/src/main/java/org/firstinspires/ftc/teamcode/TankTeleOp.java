package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.AutonLibrary;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "TeleOp")
public class TankTeleOp extends LinearOpMode {

    private TankDrive tankDrive;
    private DrivingLibrary drivingLibrary;
    private AutonLibrary autonLibrary;

    Servo leftServo;
    //Servo rightServo;

    private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);

        drivingLibrary.rotini = hardwareMap.get(DcMotor.class, "rotini");
        drivingLibrary.rotini.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftServo = hardwareMap.get(Servo.class, "left Servo");
        //rightServo = hardwareMap.get(Servo.class, "right Servo");

        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        boolean alreadyPrinted =false;
        waitForStart();

        while(opModeIsActive() && !isStopRequested() ) {
            if(!alreadyPrinted){
                telemetry.addData("status", "OKAY WE'RE IN THE LOOP");
                alreadyPrinted=true;
            }
            tankDrive.Drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.update();

            if(gamepad2.dpad_up) {
                drivingLibrary.rotini.setPower(-1); //rotini goes up
                drivingLibrary.moveRotiniUp();
            }

            if(gamepad2.y) { //test function inputs
                leftServo.setPosition(0); //makes box parallel to floor
                sleep(1000);
                leftServo.setPosition(1); //rotates box servo in a certain direction
                sleep(1000);
            }

            if(gamepad2.dpad_down) {
                drivingLibrary.rotini.setPower(0); //cancels brake
                drivingLibrary.rotini.setPower(1); //rotini goes down
                drivingLibrary.moveRotiniDown();
            }

            if(gamepad2.a) {
                leftServo.setPosition(-1);
            }

        }
    }

    public void rotiniBrake () { //turn the motor off
        drivingLibrary.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public void boxServos(int position) {
//        leftServo.setPosition(position);
//        //rightServo.setPosition(180 - position);
//    }
}
