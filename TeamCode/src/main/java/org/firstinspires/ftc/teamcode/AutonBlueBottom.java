 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SamplePipeline.LOCATION;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Blue Bottom Auton")
public class AutonBlueBottom extends LinearOpMode {

    private DrivingLibrary drivingLibrary;
    //Servo spinningArm;
    private TankDrive tankDrive;
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    public Servo boxServo;
    public DcMotor duckSpinner;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        tankDrive = new TankDrive(this);

        drivingLibrary.setSpeed(1.0);

        boxServo = hardwareMap.get(Servo.class, "boxServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");

        telemetry.addData("status", "initialized");
        telemetry.update();
        //telemetry.addData("Type", pipeline.getType());
        //telemetry.addData("Average", pipeline.getAverage());
        // telemetry.addData("Location", pipeline.getLocation());
        telemetry.update();


        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().

                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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

        /* ABBIE'S CODE
        tankDrive.rotini.setPower(0.5);//
                sleep(1000);//
                tankDrive.rotini.setPower(0);//
         */


        if (

                opModeIsActive()) {
            if (pipeline.getLocation() == LOCATION.LEFT) {
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                //spin carousel
                drivingLibrary.spinToAngle((Math.PI * 25) / 18);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(3000);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI / 4);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(2000);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI / 2);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(2000);
                drivingLibrary.brakeStop();
            } else if (pipeline.getLocation() == LOCATION.MIDDLE) {
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                //spin carousel
                drivingLibrary.spinToAngle((Math.PI * 25) / 18);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(3000);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI / 4);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(2000);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI / 2);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(2000);
                drivingLibrary.brakeStop();
            } else {
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                //spin carousel
                drivingLibrary.spinToAngle((Math.PI * 25) / 18);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(3000);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI / 4);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(2000);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI / 2);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(2000);
                drivingLibrary.brakeStop();

            }


        }
    }
}

