package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Auton Red Bottom")
public class AutonRedBottom extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    //Servo spinningArm;
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    public DcMotor boxWheels;
    public DcMotor duckSpinner;
    TankDrive tankDrive;

    int secToCarousel=1500;
    int secToHub=1000;
    int secDuckSpin=3000;
    //Servo spinningArm;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);

        boxWheels = hardwareMap.get(DcMotor.class, "boxWheels");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");

        telemetry.addData("status", "initialized");
        telemetry.update();
        //telemetry.addData("Type", pipeline.getType());
        //telemetry.addData("Average", pipeline.getAverage());
        // telemetry.addData("Location", pipeline.getLocation());
        telemetry.update();


        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        if (opModeIsActive()) {
            tankDrive.driveADistance(6, 0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI/2);

            //going long across
            tankDrive.driveADistance(9, 0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(0);

            //going to hub
            tankDrive.driveADistance(4, 0.5);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(0);

            //deilvers freight
            tankDrive.moveRotiniToAPosition(20);
            boxWheels.setPower(1);
            sleep(2000);
            boxWheels.setPower(0);
            tankDrive.moveRotiniToAPosition(0);

            if (pipeline.getLocation() == SamplePipeline.LOCATION.LEFT) {
                tankDrive.rotini.setPower(0.5);
                sleep(1000);
                tankDrive.rotini.setPower(0);
                boxWheels.setPower(1);
                sleep(700);
                boxWheels.setPower(0);
                sleep(700);
                tankDrive.rotini.setPower(-0.5);
                sleep(1000);
            }
            else if (pipeline.getLocation() == SamplePipeline.LOCATION.MIDDLE) {
                tankDrive.rotini.setPower(0.5);
                sleep(1500);
                tankDrive.rotini.setPower(0);
                boxWheels.setPower(1);
                sleep(700);
                boxWheels.setPower(0);
                sleep(700);
                tankDrive.rotini.setPower(-0.5);
                sleep(1500);
            }
            else {
                tankDrive.rotini.setPower(0.5);
                sleep(1500);
                tankDrive.rotini.setPower(0);
                boxWheels.setPower(1);
                sleep(700);
                boxWheels.setPower(0);
                sleep(700);
                tankDrive.rotini.setPower(-0.5);
                sleep(1500);
            }
            //go to parking spot
            //goes back
            tankDrive.driveADistance(-23, -0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI/2);

            //goes to carousel ish
            tankDrive.driveADistance(-51, -0.8);
            tankDrive.brakeStop();

            /*
            tankDrive.drive(-1, -1);
            sleep(1500);
            tankDrive.brakeStop();
            //spin carousel
            tankDrive.spinToAngle(Math.PI - (Math.PI / 4));
            sleep(1000);
            tankDrive.brakeStop();
            tankDrive.drive(-1, -1);
            sleep(3000);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI -(Math.PI/4));
            sleep(1000);
            tankDrive.brakeStop();
            tankDrive.drive(-1 , -1);
            sleep(2000);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI + (Math.PI / 2));
            sleep(1000);
            tankDrive.brakeStop();
            tankDrive.drive(-1, -1);
            sleep(2000);
            tankDrive.brakeStop();
            */
        }
    }
}