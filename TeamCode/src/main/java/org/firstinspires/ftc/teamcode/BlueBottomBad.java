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

@Autonomous(name = "Red Bottom Bad")
public class BlueBottomBad extends LinearOpMode {


    //Servo spinningArm;
    //OpenCvCamera webcam;
    //SamplePipeline pipeline;
    public DcMotor boxWheels;
    public DcMotor duckSpinner;
    TankDrive tankDrive;

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
        //telemetry.update();



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

        /* ABBIE'S CODE
        tankDrive.rotini.setPower(0.5);//
                sleep(1000);//
                tankDrive.rotini.setPower(0);//
         */


        if (opModeIsActive()) {
            tankDrive.driveADistance(10, 0.7);
            tankDrive.brakeStop();
            tankDrive.spinToAngle(Math.PI / 2);
            //goes to carousel ish
            tankDrive.driveADistance(-30, -0.8);
            tankDrive.brakeStop();
        }
    }
}