package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp")
public class TankTeleOp extends LinearOpMode {

    private TankDrive tankDrive;

    Servo leftServo;
    Servo rightServo;
    public DcMotor rotini;
    private int encoderValues = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);

        rotini = hardwareMap.get(DcMotor.class, "rotini");
        leftServo = hardwareMap.get(Servo.class, "left Servo");
        rightServo = hardwareMap.get(Servo.class, "right Servo");

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

            if(gamepad2.dpad_up) { //moves arm up, serves rotate, then arm moves down
                rotini.setPower(-1);
                sleep(600);
                rotini.setPower(0);
                rotiniBrake();
            }

            if(gamepad2.y) {
                boxServos(90);
                sleep(1000);
                boxServos(135);
                sleep(1000);
            }

            if(gamepad2.dpad_down) {
                rotini.setPower(0);
                rotini.setPower(1);
                sleep(600);
                rotini.setPower(0);
            }

            if(gamepad2.a) {
                boxServos(90);
            }

        }
    }

    public void rotiniBrake () { //turn the motor off
        rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void boxServos(int position) {
        leftServo.setPosition(position);
        rightServo.setPosition(180 - position);
    }
}
