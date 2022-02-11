package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.AutonLibrary;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name = "drive encoder testing")
public class DriveEncoderTest extends LinearOpMode {

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
                telemetry.addData("left value",  tankDrive.left.getCurrentPosition());
                telemetry.addData("right value",  tankDrive.right.getCurrentPosition());

                alreadyPrinted = true;
            }
            tankDrive.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tankDrive.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tankDrive.left.setPower(1);
            tankDrive.right.setPower(1);

            sleep(1000);
            tankDrive.left.setPower(0);
            tankDrive.right.setPower(1);
            tankDrive.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tankDrive.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("value", tankDrive.right.getCurrentPosition());

        }
    }
}

