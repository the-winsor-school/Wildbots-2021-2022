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


@Autonomous(name = "Auton Blue Top")
public class AutonBlueTop extends LinearOpMode {

    //Servo spinningArm;
    /*OpenCvCamera webcam;
    SamplePipeline pipeline;
    public Servo boxServo;
     */
    public DcMotor duckSpinner;
    public DcMotor boxWheels;
    TankDrive tankDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        boxWheels = hardwareMap.get(DcMotor.class, "boxWheels");
        telemetry.addData("status", "initialized");
        telemetry.update();
        /*
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
         */

        waitForStart();

        if (opModeIsActive()) {
            tankDrive.driveADistance(6, 0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(-Math.PI/2);

            tankDrive.driveADistance(18, 0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(0);

            tankDrive.driveADistance(27, 0.7);
            tankDrive.brakeStop();

            tankDrive.moveRotiniToAPosition(22);
            boxWheels.setPower(-1);
            sleep(1000);
            boxWheels.setPower(0);
            tankDrive.moveRotiniToAPosition(0);

            tankDrive.driveADistance(30, -0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(-Math.PI/2);

            tankDrive.driveADistance(40, -0.7);
            tankDrive.brakeStop();

            duckSpinner.setPower(1);
            sleep(1000);
            duckSpinner.setPower(0);

            tankDrive.spinToAngle(0);
            tankDrive.driveADistance(30, 0.7);
            tankDrive.brakeStop();
            /*
            if (pipeline.getLocation() == SamplePipeline.LOCATION.LEFT) {
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
            else if (pipeline.getLocation() == SamplePipeline.LOCATION.MIDDLE) {
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

             */
            

            /*
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(500);
            drivingLibrary.brakeStop();
            //spin carousel
            drivingLibrary.spinToAngle(Math.PI - (Math.PI / 4));
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(3000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI -(Math.PI/4));
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            drivingLibrary.spinToAngle(Math.PI + (Math.PI / 2));
            sleep(1000);
            drivingLibrary.brakeStop();
            drivingLibrary.bevelDrive(0, -1, 0);
            sleep(2000);
            drivingLibrary.brakeStop();
            */
        }
    }
}