package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.TankDriveLibrary;

@TeleOp(name = "TeleOp")
public class TankTeleOp extends LinearOpMode {

    private TankDriveLibrary tankDriveLibrary;

    Servo leftServo;
    //Servo rightServo;
    public DcMotor rotini;
    private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tankDriveLibrary = new TankDriveLibrary(this);

        rotini = hardwareMap.get(DcMotor.class, "rotini");
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
            tankDriveLibrary.Drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.update();
// rotini = motor (up and down) bc motors rotate and therefore rotini
            if(gamepad2.dpad_up) {
                rotini.setPower(-1); //rotini goes up
                sleep(600); // 1/3 of full height
                rotini.setPower(0); //stops rotini
                rotiniBrake();
                //hello world
            }

            if(gamepad2.y) { //test function inputs
                leftServo.setPosition(0); //makes box parallel to floor
                sleep(1000);
                leftServo.setPosition(1); //rotates box servo in a certain direction
                sleep(1000);
            }

            if(gamepad2.dpad_down) {
                rotini.setPower(0); //cancels brake
                rotini.setPower(1); //rotini goes down
                sleep(600);
                rotini.setPower(0); //stops rotini
            }

            if(gamepad2.a) {
                leftServo.setPosition(-1);
            }

        }
    }

    public void rotiniBrake () { //turn the motor off
        rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public void boxServos(int position) {
//        leftServo.setPosition(position);
//        //rightServo.setPosition(180 - position);
//    }
}
