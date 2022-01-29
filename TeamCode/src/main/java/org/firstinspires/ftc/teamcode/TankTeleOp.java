//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.libraries.AutonLibrary;
//import org.firstinspires.ftc.libraries.DrivingLibrary;
//import org.firstinspires.ftc.teamcode.TankDrive;
//
//@TeleOp(name = "TeleOp")
//public class TankTeleOp extends LinearOpMode {
//
//    private TankDrive tankDrive;
//
//
//    public Servo leftServo;
//    public Servo rightServo;
//    public Servo cappingServo;
//
//    public DcMotor duckSpinner;
//
//
//    //private int encoderValues = 0;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        tankDrive = new TankDrive(this);
//
//        leftServo = hardwareMap.get(Servo.class, "left Servo");
//        rightServo = hardwareMap.get(Servo.class, "right Servo");
//        cappingServo = hardwareMap.get(Servo.class, "cap Servo");
//        duckSpinner = hardwareMap.get(DcMotor.class, "Duck Spinning Motor");
//
//        telemetry.addData("status", "BAAAAAAAH initialized");
//        telemetry.update();
//        boolean alreadyPrinted = false;
//        int rotiniTarget = 0;
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
//            if (!alreadyPrinted) {
//                telemetry.addData("status", "OKAY WE'RE IN THE LOOP");
//                alreadyPrinted = true;
//            }
//            telemetry.addData("rotini", "current height =" + tankDrive.getRotiniHeight());
//            telemetry.addData("rotini", "target height =" + rotiniTarget);
//            tankDrive.Drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
//            // use joysticks for Tankalicious teleop
//            telemetry.update();
//
//
//            if (rotiniTarget > tankDrive.getRotiniHeight()) {
//                tankDrive.rotini.setPower(-1);
//                telemetry.addData("status", "raising rotini");
//            } else if (rotiniTarget < tankDrive.getRotiniHeight()) {
//                tankDrive.rotini.setPower(1);
//                telemetry.addData("status", "lowering rotini");
//            } else {
//                tankDrive.rotini.setPower(0);
//            }
//            if (gamepad2.dpad_up) {
//                // up arrow
//                rotiniTarget = tankDrive.moveRotiniUp();
//                tankDrive.rotini.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                // moves arm up
//            }
//
//            if (gamepad2.dpad_down) {
//                // down arrow
//                rotiniTarget = tankDrive.moveRotiniDown();
//                tankDrive.rotini.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                // moves arm down
//            }
//
//            if (gamepad2.b) {
//                // letter b
//                duckSpinner.setPower(1);
//                sleep(100); //CHANGE for amount of time to spin duck off
//                // spins duck spinner
//            }
//
//            if (gamepad2.y) { //test function inputs
//                // letter y
//                leftServo.setPosition(0);
//                rightServo.setPosition(0);
//                sleep(1000);
//                leftServo.setPosition(1);
//                rightServo.setPosition(-1);
//                sleep(1000);
//                // intake
//                // moves one servo in one direction and the other in the other direction
//            }
//
//            if (gamepad2.a) { //test function inputs
//                // letter a
//                leftServo.setPosition(0);
//                rightServo.setPosition(0);
//                sleep(1000);
//                leftServo.setPosition(-1);
//                rightServo.setPosition(1);
//                sleep(1000);
//                // outtake
//            }
//
//            if (gamepad2.right_bumper) {
//                cappingServo.setPosition(45);
//                // moves capping servo 45 degrees to the right
//            }
//
//            if (gamepad2.left_bumper) {
//                cappingServo.setPosition(-45);
//                // moves capping servo 45 degrees to the left(?)
//            }
//
//        }
//    }
//
//    public void rotiniBrake () { //turn the motor off
//        tankDrive.rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void boxServos(int position) {
//        leftServo.setPosition(position);
//    }
//}