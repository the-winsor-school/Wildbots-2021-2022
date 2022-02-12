 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SamplePipeline.LOCATION;

//import org.firstinspires.ftc.libraries.TankDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Blue Bottom Auton")
public class AutonBlueBottom extends LinearOpMode {


    //Servo spinningArm;
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    public Servo boxServo;
    public DcMotor duckSpinner;
    TankDrive tankDrive;

    int secToCarousel=1500;
    int secToHub=1000;
    int secDuckSpin=3000;

    //initializing
    @Override
    public void runOpMode() throws InterruptedException {

        tankDrive = new TankDrive(this);

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


        if (opModeIsActive()) {

                tankDrive.spinToAngle(-Math.PI/2);
                //go to the carousel
                tankDrive.drive(1,1);
                sleep(secToCarousel);
                tankDrive.brakeStop();

                //spin carousel
                duckSpinner.setPower(1);
                sleep(secDuckSpin);
                duckSpinner.setPower(0);

                //go to the alliance hub?
                tankDrive.drive(-1,-1);
                sleep(secToCarousel*2);
                tankDrive.brakeStop();

                tankDrive.spinToAngle(0);
                tankDrive.drive(1, 1);
                sleep(secToHub);
                tankDrive.brakeStop();

            if (pipeline.getLocation() == LOCATION.LEFT) {
                tankDrive.rotini.setPower(0.5);
                sleep(1000);
                tankDrive.rotini.setPower(0);
                boxServo.setPosition(1);
                sleep(700);
                boxServo.setPosition(0);
                sleep(700);
                tankDrive.rotini.setPower(-0.5);
                sleep(1000);
            }
            else if (pipeline.getLocation() == LOCATION.MIDDLE) {
                tankDrive.rotini.setPower(0.5);
                sleep(1500);
                tankDrive.rotini.setPower(0);
                boxServo.setPosition(1);
                sleep(700);
                boxServo.setPosition(0);
                sleep(700);
                tankDrive.rotini.setPower(-0.5);
                sleep(1500);
            }
            else {
                tankDrive.rotini.setPower(0.5);
                sleep(1500);
                tankDrive.rotini.setPower(0);
                boxServo.setPosition(1);
                sleep(700);
                boxServo.setPosition(0);
                sleep(700);
                tankDrive.rotini.setPower(-0.5);
                sleep(1500);
            }
            //go to parking spot
            tankDrive.drive(-1, -1);
            sleep(secToHub);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(-Math.PI/2);
            tankDrive.drive(-1, -1);
            sleep(secToCarousel*2);
            //should be back at carousel
            tankDrive.brakeStop();
            tankDrive.spinToAngle(0);
            tankDrive.drive(1,1);
            sleep(1000);
            tankDrive.brakeStop();


                /*
                tankDrive.spinToAngle((Math.PI * 25) / 18);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(3000);
                tankDrive.brakeStop();
                tankDrive.spinToAngle(Math.PI / 4);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(2000);
                tankDrive.brakeStop();
                tankDrive.spinToAngle(Math.PI / 2);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(2000);
                tankDrive.brakeStop();


            } else if (pipeline.getLocation() == LOCATION.MIDDLE) {
                tankDrive.drive(1,1);
                sleep(1500);
                tankDrive.brakeStop();
                //spin carousel
                tankDrive.spinToAngle((Math.PI * 25) / 18);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(3000);
                tankDrive.brakeStop();
                tankDrive.spinToAngle(Math.PI / 4);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(2000);
                tankDrive.brakeStop();
                tankDrive.spinToAngle(Math.PI / 2);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(2000);
                tankDrive.brakeStop();
            } else {
                tankDrive.drive(1,1);
                sleep(1500);
                tankDrive.brakeStop();
                //spin carousel
                tankDrive.spinToAngle((Math.PI * 25) / 18);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(3000);
                tankDrive.brakeStop();
                tankDrive.spinToAngle(Math.PI / 4);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(2000);
                tankDrive.brakeStop();
                tankDrive.spinToAngle(Math.PI / 2);
                sleep(1000);
                tankDrive.brakeStop();
                tankDrive.drive(1,1);
                sleep(2000);
                tankDrive.brakeStop();

            }
            */



        }
    }
}

