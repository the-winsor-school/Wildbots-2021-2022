package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@TeleOp(name = "RevampingRoverRuckus", group = "Concept")

/**
 * Created by lamanwyner on 2/9/19.
 */
public class RevampingRoverRuckus extends LinearOpMode {
    DrivingLibrary drivingLibrary; // renaming the drivinglibrary

    // DcMotor latchArm;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite"; // DO WE NEED TO DOWNLOAD ROVERRUCKUS FILES?
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AYtzJiX/////AAABmdYFe9BmNk+ql9Whza+MyWA8JYo1KDpmYAQYA/mdTQN3Kt9J13kde2C5eYuGN725m11GpnoMomwiqCvODM+eN/9NK3ERjoUKyJdBJtiIPQb5s7XzqpJ7SDyfLuWRh97kNvJyuMS1dUwS8mHoaXCdocSy+9O7ufi90XbGJbPcq63GrkgYxzcdnfVxKFOGbqnYcXU3vMrqNAQHpgWGtmiYpjUmpJ1gaGCjMXIUq6BclVWgaqirlRf78FjO7Z+2I83+Z8Q+DUTnVmlyUKqG4+F+doEfFhqIqH+eeKl7bN6M2cQBlJNOaXmCrTGylB45k/2mDitSUMD4I+g/64+hBIepHTbcgWjlFQAog38GQJ5gP0p6";

    private VuforiaLocalizer vuforia; // Vuforia Library gives opMode easy access to the webcam
    private TFObjectDetector tfod; // An object that contains a trained model and builtin functions for identifying game elements

    private int goldPos;

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();

        /*drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setMode(DrivingMode.BRAKE_STOP);
        drivingLibrary.setSpeed(1);
        */

        //latchArm = hardwareMap.get(DcMotor.class, "latchArm");

        goldPos = -1;

        waitForStart();

        if (opModeIsActive()) {
            tfod.activate();

            while (opModeIsActive()) {
                // latchArm.setPower(1);
                // sleep(23000);
                // latchArm.setPower(0);

                /* drivingLibrary.strafe(.65f, 0);
                sleep(500);
                drivingLibrary.brakeStop();
                sleep(500);
                */


                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // if (updatedRecognitions.size() == 2) {
                        /*
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                         */
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Cargo Detected", goldPos);
                            }
                                // goldMineralX = (int) recognition.getLeft();
                            }
                            /* else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                             */
                       /* if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            goldPos = 3;
                            drivingLibrary.strafe(0, -.65f);
                            sleep(2000);
                            drivingLibrary.strafe(-.65f, 0);
                            sleep(2000);
                            drivingLibrary.strafe(0, -.65f);
                            sleep(2000);
                        } else if (goldMineralX != -1 && silverMineral1X != 1) {
                            if (goldMineralX < silverMineral1X) {
                                goldPos = 1;
                                drivingLibrary.strafe(0, 1);
                                sleep(2000);
                            } else {
                                goldPos = 2;
                                drivingLibrary.strafe(0, 1);
                                sleep(2000);
                                drivingLibrary.strafe(1, 0);
                                sleep(2000);
                                drivingLibrary.strafe(0, 1);
                                sleep(2000);
                            }
                        } */

                        // drivingLibrary.brakeStop();
                        // telemetry.addData("Gold Mineral Position", goldPos);
                    }
                    telemetry.update();
                }
            }

            tfod.shutdown();
        }
   // }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}