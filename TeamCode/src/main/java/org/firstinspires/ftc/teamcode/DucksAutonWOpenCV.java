package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//webcam stuff
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

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "DucksAutonWOpenCV")
public class DucksAutonWOpenCV extends LinearOpMode {

    //webcam stuff
    OpenCvCamera webcam;
    opencvexample.SamplePipeline pipeline;


    //private DrivingLibrary drivingLibrary;
    private TankDrive tankDrive;

    //public CRServo boxServo;

    //public Servo cappingServo;

    public DcMotor leftIntakeSpinner;
    public DcMotor rightIntakeSpinner;
    public DcMotor frontIntakeSpinner;
    AnalogInput forceSensitiveResistor;

    public int servoPosition;

    public DcMotor duckSpinner;

    //initializing

    public void turn90Right() {
        TankDrive tankDrive= new TankDrive(this);
        tankDrive.drive(0.5,-0.5);
        sleep(1500);
        tankDrive.brakeStop();
    }
    public void turn90Left() {
        TankDrive tankDrive= new TankDrive(this);
        tankDrive.drive(-0.5,0.5);
        sleep(1500);
        tankDrive.brakeStop();
    }
    @Override
    public void runOpMode() throws InterruptedException {
//        drivingLibrary = new DrivingLibrary(this);
//        drivingLibrary.setSpeed(1.0);

        //webcam stuff
        DucksAutonWOpenCV.SamplePipeline samplePipeline = new DucksAutonWOpenCV.SamplePipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new opencvexample.SamplePipeline();
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


        TankDrive tankDrive= new TankDrive(this);
        //boxServo = hardwareMap.get(CRServo.class, "boxServo");
        //cappingServo = hardwareMap.get(Servo.class, "cappingServo");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        leftIntakeSpinner = hardwareMap.get(DcMotor.class, "leftIntakeSpinner");
        rightIntakeSpinner = hardwareMap.get(DcMotor.class, "rightIntakeSpinner");
        frontIntakeSpinner = hardwareMap.get(DcMotor.class, "frontIntakeSpinner");



        double currentForce;
        forceSensitiveResistor = hardwareMap.get(AnalogInput.class, "Force Sensitive Resistor");
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        boolean alreadyPrinted = false;
        int rotiniTarget = 0;

        //spinningArm = hardwareMap.get(Servo.class, "Carousel Spinning Arm");

        waitForStart();

        if (opModeIsActive()) {
            //camera reads value
            if (samplePipeline.average1 > samplePipeline.THRESHOLD2) {
                samplePipeline.location = SamplePipeline.LOCATION.LEFT;
            }
             if (samplePipeline.average2 > samplePipeline.THRESHOLD2) {
                 samplePipeline.location = SamplePipeline.LOCATION.MIDDLE;
            }
             if (samplePipeline.average3 > samplePipeline.THRESHOLD2) {
                 samplePipeline.location = SamplePipeline.LOCATION.RIGHT;
             }
            tankDrive.drive(1,1);
            sleep(300);
            tankDrive.brakeStop();
            //turn right
            turn90Right();
            tankDrive.drive(1,1);
            sleep(450);
            tankDrive.brakeStop();
            turn90Left();
            tankDrive.drive(1,1);
            sleep(700);
            tankDrive.brakeStop();

            sleep(700);
            //outake:
            // if location = left, drops freight in correct left
            // else if middle, middle
            // else, top

            /*if (samplePipeline.location = SamplePipeline.LOCATION.LEFT){
                tankDrive.rotini.setPower(0.5);
                tankDrive.getRotiniHeight();


            }*/
            turn90Right();
            tankDrive.drive(-1,-1);
            sleep(2000);
            tankDrive.brakeStop();
            turn90Left();
            tankDrive.drive(1,1);
            sleep(500);
            tankDrive.brakeStop();
            sleep(500);
            duckSpinner.setPower(1);
            sleep(1000);
            //spin ducks
            tankDrive.drive(1,1);
            sleep(500);
            tankDrive.brakeStop();

        }
    }

    //webcam stuff
    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        private static final int THRESHOLD1 = 107; // yellow < 107 < white
        private static final int THRESHOLD2 = 140; // white < 140 < purple

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
        private volatile LOCATION locations;

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

            /*if (average1 > THRESHOLD2) {
                location = LOCATION.LEFT;
            }
            if (average2 > THRESHOLD2) {
                location = LOCATION.MIDDLE;
            }
            if (average3 > THRESHOLD2) {
                location = LOCATION.RIGHT;
            }*/

            return input;
        }

        public TYPE getType() {
            return type;
        }
        public String getAverage() {
            return String.format("average #1: %d \n average #2: %d \n average #3: %d", average1, average2, average3);
            // %d: d means int; %d will act as a variable that the variable (average1 for the first %d) listed later will replace)
        }
        public String getLocation() {
            return String.format("Location: ", location);
        }
        public enum TYPE { //enum = variable with set output values
            BALL, CUBE, BEZ //bez for embezzlemnet (aka the team object)
        }
        public enum LOCATION {
            LEFT, MIDDLE, RIGHT
        }

    }

}