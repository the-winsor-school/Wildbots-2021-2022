//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.libraries.DrivingLibrary;
//
//@Autonomous(name = "TankTick")
//public class HowManyTicksInATankTread extends LinearOpMode{
//    private TankDrive tankDrive;
//
//    public void runOpMode() throws InterruptedException {
//        TankDrive tankDrive= new TankDrive(this);
//        telemetry.addData("status", "initialized");
//        telemetry.update();
//        waitForStart();
//        tankDrive.left.setTargetPosition(100);
//        tankDrive.left.setTargetPosition(100);
//        tankDrive.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        tankDrive.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (opModeIsActive()){
//            tankDrive.left.setPower(0.25);
//            tankDrive.right.setPower(0.25);
//
//            // wait while opmode is active and left motor is busy running to position.
//
//            while (opModeIsActive() && tankDrive.left.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//            {
//                telemetry.addData("left", tankDrive.left.getCurrentPosition() + "  busy=" + tankDrive.left.isBusy());
//                telemetry.addData("right", tankDrive.right.getCurrentPosition() + "  busy=" + tankDrive.right.isBusy());
//                telemetry.update();
//                idle();
//            }
//
//            // set motor power to zero to turn off motors. The motors stop on their own but
//            // power is still applied so we turn off the power.
//
//            tankDrive.left.setPower(0.0);
//            tankDrive.right.setPower(0.0);
//
//            telemetry.addData("left Final", tankDrive.left.getCurrentPosition() + "  busy=" + tankDrive.left.isBusy());
//            telemetry.addData("right Final", tankDrive.right.getCurrentPosition() + "  busy=" + tankDrive.right.isBusy());
//        }
//    }
//}
