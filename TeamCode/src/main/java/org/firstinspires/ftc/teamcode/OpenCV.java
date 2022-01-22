<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpenCV.java

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class OpenCV extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
           // telemetry.addData("Average #2", pipeline.getAverage());
           // telemetry.addData("Average #3", pipeline.getAverage());
            telemetry.addData("Location", pipeline.getLocation());
            telemetry.update();
            sleep(50);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        //private static final int THRESHOLD1 = 107; // yellow < 107 < white
        private static final int THRESHOLD2 = 138; // white < 140 < purple

//random x values. NEED TO TEST!
//coordinates for each region
        //region 1
        Point topLeft1 = new Point(50, 50);
        Point bottomRight1 = new Point(100, 100);
        //region 2
        Point topLeft2 = new Point(150, 50);
        Point bottomRight2 = new Point(200, 100);
        //region 3
        Point topLeft3 = new Point(250, 50);
        Point bottomRight3 = new Point(300, 100);
//mat = matrix
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;
        Mat YCrCb = new Mat(); //color data
        Mat Cb = new Mat();


        private volatile int average1;
        private volatile int average2;
        private volatile int average3;
        private volatile TYPE type = TYPE.BALL;
        private volatile LOCATION location;

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            region1_Cb = Cb.submat(new Rect(topLeft1, bottomRight1));
            region2_Cb = Cb.submat(new Rect(topLeft2, bottomRight2));
            region3_Cb = Cb.submat(new Rect(topLeft3, bottomRight3));        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            average1 = (int) Core.mean(region1_Cb).val[0];
            average2 = (int) Core.mean(region2_Cb).val[0];
            average3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(input, topLeft1, bottomRight1, BLUE, 2);
            Imgproc.rectangle(input, topLeft2, bottomRight2, BLUE, 2);
            Imgproc.rectangle(input, topLeft3, bottomRight3, BLUE, 2);

            if (average1 > THRESHOLD2) {
                location = LOCATION.LEFT;

            }
            if (average2 > THRESHOLD2) {
                location = LOCATION.MIDDLE;

            }
            if (average3 > THRESHOLD2) {
                location = LOCATION.RIGHT;

            }


            return input;
        }

        public TYPE getType() {
            return type;
        }

        public String getAverage() {
            return String.format(" #1: %d \n Average #2: %d \n Average #3: %d", average1, average2, average3);
            // %d: d means int; %d will act as a variable that the variable (average1 for the first %d) listed later will replace)
        }
        public String getLocation(){
            return String.format("%s", location);
            //Output = LOCATION.LEFT, LOCATION.MIDDLE, or LOCATION.RIGHT
        }
        public enum TYPE { //enum = variable with set output values
            BALL, CUBE, BEZ //bez for embezzlement (aka the team object)
        }
        public enum LOCATION {
            LEFT, MIDDLE, RIGHT
        }
    }
}
