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

@Autonomous(name = "Auton Red Top")
public class AutonRedTop extends LinearOpMode {

   public DcMotor duckSpinner;
    public DcMotor boxWheels;
    TankDrive tankDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);
        //boxServo = hardwareMap.get(Servo.class, "boxServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");

        telemetry.addData("status", "initialized");
        telemetry.update();
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // webcam = OpenCvCameraFactory.getInstance().

                //createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //webcam.setPipeline(pipeline);

/*
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
            tankDrive.spinToAngle(-Math.PI/2);
            tankDrive.driveADistance(18,0.5);
            //sleep(secToHubVert);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(0);
            tankDrive.driveADistance(27,0.5);
            //sleep(secToHubHoriz);
            tankDrive.brakeStop();
            moveRotiniToAPosition(22);
            boxWheels.setPosition(1);
            sleep(700);
            boxWheels.setPosition(0);
            tankDrive.driveADistance(30,0.5);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(-Math.PI/2);
            tankDrive.driveADistance(40,0.5);
            tankDrive.brakeStop();
            duckSpinner.setPower(1);
            sleep(1500);
            duckSpinner.setPower(0);
            tankDrive.spinToAngle(0);
            tankDrive.driveADistance(30,0.5);
            tankDrive.brakeStop();
        }
    }
}