//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.enums.DrivingMode;
//import org.firstinspires.ftc.libraries.DrivingLibrary;
//
//import java.util.ArrayList;
//
//
//@TeleOp(name  = "Liftie TeleOp Draft", group = "Finished")
//public class LiftieTeleOpDraft extends LinearOpMode {
//    //drive train
//    DrivingLibrary drivingLibrary;
//    int drivingMode;
//
//    public DcMotor joint;
//    //public DcMotor spinner;
//    public Servo leftServo;
//    public Servo rightServo;
//
//    public DcMotor duckSpinner;
//    //public Servo cappingServo;
//
//    public void runOpMode() throws InterruptedException {
//        //set up our driving library
//        drivingLibrary = new DrivingLibrary(this);
//        drivingLibrary.setSpeed(1);
//        drivingMode = 0;
//        drivingLibrary.setMode(drivingMode);
//
//        DcMotor intakeSpinner;
//        Servo cappingServo;
//
//        joint = hardwareMap.get(DcMotor.class, "joint");
//        intakeSpinner = hardwareMap.get(DcMotor.class, "spinner");
//        leftServo = hardwareMap.get(Servo.class, "left Servo");
//        rightServo = hardwareMap.get(Servo.class, "right Servo");
//        duckSpinner = hardwareMap.get(DcMotor.class, "duck spinning motor");
//        cappingServo = hardwareMap.get(Servo.class, "capping Servo");
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            //driving
//            drivingLibrary.bevelDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//
//            // switching braking modes
//            if (gamepad1.b) {
//                // letter b
//                drivingMode++;
//                drivingMode %= DrivingMode.values().length;
//                drivingLibrary.setMode(drivingMode);
//            }
//
//            joint.setPower(gamepad2.right_stick_y); //could do 3 increments instead of continuous
//
//            if(gamepad1.dpad_up) { //rotating servos/intake box up/down
//                // up button
//                double position = leftServo.getPosition();
//                leftServo.setPosition(position += 0.05); //servo is coded in 0.05 increments (0-1)
//                position = rightServo.getPosition();
//                rightServo.setPosition(position -= 0.05); //may have to switch + and - for right and left servos
//            }
//
//            if(gamepad1.dpad_down) {
//                // down button
//                double position = leftServo.getPosition();
//                leftServo.setPosition(position -= 0.05);
//                position = rightServo.getPosition();
//                rightServo.setPosition(position += 0.05);
//            }
//
//            if(gamepad1.a) {
//                // letter a
//                // duck spinner
//                duckSpinner.setPower(1);
//                sleep(100); //CHANGE for amount of time to spin duck off
//            }
//
//            if (gamepad1.right_bumper) {
//                cappingServo.setPosition(45);
//            }
//
//            if (gamepad1.left_bumper) {
//                cappingServo.setPosition(90);
//            }
//
//
///*
//            rotini.setPower(gamepad1.right_stick_y);
//
//            if(gamepad1.x) { //brake stop (not float)
//            // x button
//                rotiniBrake();
//            }
//
//            if(gamepad1.y) { //float stop
//            // y button
//                rotini.setPower(0);
//            }
//
//             */
//
//            telemetry.addData("Status", "Running"); //prints to phone
//            telemetry.addData("Brake Mode", drivingLibrary.getMode());
//
//            telemetry.update(); //makes actually print
//        }
//    }
//
////    public void servosUp () { //sets position at a certain angle
////        leftServo.setPosition(1); //please recalibrate and test
////        rightServo.setPosition(0);
////    }
////
////    public void servosDown () {
////        leftServo.setPosition(0);
////        rightServo.setPosition(1);
////    }
//
//}