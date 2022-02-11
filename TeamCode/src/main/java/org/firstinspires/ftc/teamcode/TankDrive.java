package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

public class TankDrive {
    // hardware variables
    public DcMotor left;
    public DcMotor right;
    public DcMotor rotini;
    public org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline pipeline;
    AnalogInput forceSensitiveResistor;
    private BNO055IMU imu; //gyroscope in rev hub
    private Orientation angles;
    private Acceleration gravity;

    private DcMotor[] allMotors;
    private HardwareMap hardwareMap;
    private OpMode opMode;

    public TankDrive(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;


        left = hardwareMap.tryGet(DcMotor.class, "left");
        right = hardwareMap.tryGet(DcMotor.class, "right");
        rotini = hardwareMap.get(DcMotor.class, "rotini");
        forceSensitiveResistor = hardwareMap.tryGet(AnalogInput.class, "Force Sensitive Resistor");
    }

    public void drive(double l, double r) {
        left.setPower(l);
        right.setPower(-r);
    }

    public double getRotiniHeight() {
        return (rotini.getCurrentPosition() / 28) * 3.14 * 0.5;
    }

    public int convertToRotini(int num) {
        return (int) (num * 28 / (Math.PI * 0.5));
    }

    public int convertToTread(int num) {
        return (int) (num * 28 / (Math.PI * 0.7));
    } //no idea what the conversion factor actually is yet :(

    public int moveRotiniUp() {
        //28 counts per rev for this motor
        //diameter of wheel is like half an inch :O
        //goes up 7 in
        double position = getRotiniHeight();
        int target = 0;
        if (position < 21) {
            if (position < 7) {
                target = convertToRotini(7);

            } else if (position < 14) {
                target = convertToRotini(14);

            } else {
                target = convertToRotini(21);
            }
            rotini.setTargetPosition(target);
            return target;
        }
        return 21;

    }

    //move the rotini down
    public int moveRotiniDown() {
        //28 counts per rev for this motor
        //diameter of wheel is like half an inch :T
        //goes up 7 in
        int target = 0;
        double position = getRotiniHeight();
        if (position > 0) {
            if (position > 14) {
                target = convertToRotini(14);
            } else if (position > 7) {
                target = convertToRotini(7);
            } else {
                target = convertToRotini(0);
            }
            rotini.setTargetPosition(target);
            return target;

        }
        return 21;
    }

    //theoretically could be used to measure the weight and automatically move rotini height, idk if that's what we want yet though
    public int weightToRotini(int num) {
        double currentForce = forceSensitiveResistor.getVoltage();
        // integrating FSR
        int target = 0;
        if (currentForce > 0.113 && currentForce < 0.169) { //light
            target = ((int) (1 * 28 / (Math.PI * 0.5)));
        } else if (currentForce > 0.189 && currentForce < 0.224) { //medium
            target = ((int) (7 * 28 / (Math.PI * 0.5)));
        } else if (currentForce > 0.235 && currentForce < 0.278) { //heavy
            target = ((int) (14 * 28 / (Math.PI * 0.5)));
        }
        return target;
    }

    //i got tired of writing the same thing over again so this one will actually the target position to rotini numbers
    public int moveRotiniToAPosition(int num) {
        int target = 0;
        switch (num) {
            case 1:
                target = convertToRotini(0);
                break;
            case 2:
                target = convertToRotini(7);
                break;
            case 3:
                target = convertToRotini(14);
                break;
        }
        return target;
    }

    //just so it exists so I can write pseudo code with it lol
    public int detectDuck() {
        //do stuff
        return 0;
    }

    public void driveADistance(int dist, double motorPower) {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(convertToTread(dist));
        right.setTargetPosition(convertToTread(dist));
        left.setPower(motorPower);
        right.setPower(motorPower);
        //still have to set motor power to 0 after lol sorry

    }

    // get current robot angle (radians)
    public double getIMUAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    public void spinToAngle(double angle) {
        double goalAngle = getIMUAngle() + angle;
        while (Math.abs(angle - getIMUAngle()) > .15) {
            if (angle > 0) {
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setPower(0.25f);
            } else {
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right.setPower(0.25f);
            }
            opMode.telemetry.update();
        }
        brakeStop();
        //targetAngle = getIMUAngle();
    }

    public void brakeStop() {
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setPower(0);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setPower(0);

    }

    public void resetRotini() {
        rotini.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotini.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}


    class OpenCV2 extends LinearOpMode {
        OpenCvCamera webcam;
        org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline pipeline;

        @Override
        public void runOpMode() {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            pipeline = new org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline();
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


        }
    }
        class SamplePipeline extends OpenCvPipeline {
            private static final Scalar BLUE = new Scalar(0, 0, 255);

            //private static final int THRESHOLD1 = 107; // yellow < 107 < white
            private static final int THRESHOLD2 = 138; // white < 140 < purple

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
            private volatile org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.TYPE type = org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.TYPE.BALL;
            private volatile org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.LOCATION location;

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
                    location = org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.LOCATION.LEFT;

                }
                if (average2 > THRESHOLD2) {
                    location = org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.LOCATION.MIDDLE;

                }
                if (average3 > THRESHOLD2) {
                    location = org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.LOCATION.RIGHT;

                }


                return input;
            }

            public org.firstinspires.ftc.teamcode.OpenCV.SamplePipeline.TYPE getType() {
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


