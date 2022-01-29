//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.libraries.DrivingLibrary;
//
//@Disabled
//@Autonomous(name = "Blue Bottom Auton")
//public class AutonBlueBottom extends LinearOpMode {
//
//    private DrivingLibrary drivingLibrary;
//    //Servo spinningArm;
//    private TankDrive tankDrive;
//    private OpenCV.SamplePipeline pipeline;
//
//    //initializing
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drivingLibrary = new DrivingLibrary(this);
//        tankDrive = new TankDrive (this);
//
//        drivingLibrary.setSpeed(1.0);
//        pipeline = new org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline();
//        telemetry.addData("status", "initialized");
//        telemetry.update();
//        telemetry.addData("Type", tankDrive.pipeline.getType());
//        telemetry.addData("Average", tankDrive.pipeline.getAverage());
//        telemetry.addData("Location", tankDrive.pipeline.getLocation());
//        telemetry.update();
//        }
//
//        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");
//
//        //waitForStart();
//
//        //if (opModeIsActive()) {
//
//            drivingLibrary.bevelDrive(0, -1, 0);
//            sleep(1500);
//            drivingLibrary.brakeStop();
//            //spin carousel
//            drivingLibrary.spinToAngle((Math.PI * 25) / 18);
//            sleep(1000);
//            drivingLibrary.brakeStop();
//            drivingLibrary.bevelDrive(0, -1, 0);
//            sleep(3000);
//            drivingLibrary.brakeStop();
//            drivingLibrary.spinToAngle(Math.PI/4);
//            sleep(1000);
//            drivingLibrary.brakeStop();
//            drivingLibrary.bevelDrive(0, -1, 0);
//            sleep(2000);
//            drivingLibrary.brakeStop();
//            drivingLibrary.spinToAngle(Math.PI/2);
//            sleep(1000);
//            drivingLibrary.brakeStop();
//            drivingLibrary.bevelDrive(0, -1, 0);
//            sleep(2000);
//            drivingLibrary.brakeStop();
//
//        }
//    }
//}