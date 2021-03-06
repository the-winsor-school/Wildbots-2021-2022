package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
@TeleOp(name = "BarcodeWithTensorFlow", group = "Concept")
// @Disabled
public class BarcodeWithTensorFlow extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next lin
         * e, between the double quotes.
         */
        private static final String VUFORIA_KEY = "AYtzJiX/////AAABmdYFe9BmNk+ql9Whza+MyWA8JYo1KDpmYAQYA/mdTQN3Kt9J13kde2C5eYuGN725m11GpnoMomwiqCvODM+eN/9NK3ERjoUKyJdBJtiIPQb5s7XzqpJ7SDyfLuWRh97kNvJyuMS1dUwS8mHoaXCdocSy+9O7ufi90XbGJbPcq63GrkgYxzcdnfVxKFOGbqnYcXU3vMrqNAQHpgWGtmiYpjUmpJ1gaGCjMXIUq6BclVWgaqirlRf78FjO7Z+2I83+Z8Q+DUTnVmlyUKqG4+F+doEfFhqIqH+eeKl7bN6M2cQBlJNOaXmCrTGylB45k/2mDitSUMD4I+g/64+hBIepHTbcgWjlFQAog38GQJ5gP0p6";

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        private VuforiaLocalizer vuforia;

        /**
         * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
         * Detection engine.
         */
        private TFObjectDetector tfod;

        @Override
        public void runOpMode() {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();

           /* if (ClassFactory.getInstance().createTFObjectDetector(TFObjectDetector.Parameters parameters,
                    VuforiaLocalizer vuforiaLocalizer)) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }*/

            /** Wait for the game to begin */
            telemetry.addData(">", "Press play and you're cool");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                /** Activate TensorFlow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                    // The TensorFlow software will scale the input images from the camera to a lower resolution.
                    // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                    // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                    // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                    // should be set to the value of the images used to create the TensorFlow Object Detection model
                    // (typically 16/9).
                    tfod.setZoom(2.5, 16.0/9.0);
                }

                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }

        /**
         * Initialize the Vuforia localization engine.
         */
        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

              parameters.cameraDirection = CameraDirection.BACK;
              parameters.vuforiaLicenseKey = VUFORIA_KEY;
            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
            tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }
    }

