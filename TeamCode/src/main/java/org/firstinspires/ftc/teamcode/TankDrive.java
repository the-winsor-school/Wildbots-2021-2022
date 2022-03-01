package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TankDrive {
    // hardware variables
    public DcMotor left;
    public DcMotor right;
    public DcMotor rotini;
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
        left.setDirection(DcMotor.Direction.REVERSE);
        right = hardwareMap.tryGet(DcMotor.class, "right");
        rotini = hardwareMap.get(DcMotor.class, "rotini");
        rotini.setDirection(DcMotor.Direction.REVERSE);
        forceSensitiveResistor = hardwareMap.tryGet(AnalogInput.class, "FSR");

        //for imu set up
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void drive(double l, double r) {
        left.setPower(l); //reversed these (took out - sign)
        right.setPower(r); //same
    }

    public double getRotiniHeight() {
        return (rotini.getCurrentPosition()/420);
    }

    public int convertToRotini(int num) {
        return num*420;
    }

    public int convertToTread(int num) {
        return (int) (num * 28 / (Math.PI * 0.7));
    } //no idea what the conversion factor actually is yet :(

    public void moveRotiniUp(int pos) {

        int initPos=rotini.getCurrentPosition();
        int curPos=rotini.getCurrentPosition();
        //if(getRotiniHeight()<12){
            while(curPos-initPos>convertToRotini(pos)){
                rotini.setPower(-0.5);
                curPos=rotini.getCurrentPosition();
                opMode.telemetry.addData("initial rotini", initPos);
                opMode.telemetry.addData("rotini", curPos-initPos);
                opMode.telemetry.update();
            }
            rotini.setPower(0);
            rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //}
    }

    //move the rotini down
    public void moveRotiniDown(int pos) {
        int initPos=rotini.getCurrentPosition();
        int curPos=rotini.getCurrentPosition();
        //if(getRotiniHeight()>4){
            while(curPos-initPos<convertToRotini(pos)){
                rotini.setPower(0.5);
                curPos=rotini.getCurrentPosition();
                opMode.telemetry.addData("initial rotini", initPos);
                opMode.telemetry.addData("rotini", curPos-initPos);
                opMode.telemetry.update();
            }
            rotini.setPower(0);
            rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //}
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
    public void moveRotiniToAPosition(int num) {
        brakeStop();
        int curPos=rotini.getCurrentPosition();
        if(convertToRotini(num)>curPos){
            while(curPos<convertToRotini(num)){
                rotini.setPower(1);
                curPos=rotini.getCurrentPosition();
                opMode.telemetry.update();
            }
        }
        if(convertToRotini(num)<curPos){
            while(curPos>convertToRotini(num)){
                rotini.setPower(-0.7);
                curPos=rotini.getCurrentPosition();
            }
        }
        rotini.setPower(0);
        rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //just so it exists so I can write pseudo code with it lol
    public int detectDuck() {
        //do stuff
        return 0;
    }

    public double inchToTankTick(double inch){
        return inch *67.3;
    }

    public void driveADistance(double dist, double motorPower) {
        int rightInitPos=right.getCurrentPosition();
        int rightCurPos=right.getCurrentPosition();
        opMode.telemetry.addData("right init", rightInitPos);
        opMode.telemetry.update();
        if(inchToTankTick(dist)>0){
            while(rightCurPos-rightInitPos<inchToTankTick(dist)){
                right.setPower(motorPower);
                left.setPower(motorPower);
                opMode.telemetry.addData("right current", rightCurPos-rightInitPos);
                opMode.telemetry.addData("goal", inchToTankTick(dist));
                opMode.telemetry.update();
                rightCurPos=right.getCurrentPosition();
            }
        }
        if(inchToTankTick(dist)<0){
            while(rightCurPos-rightInitPos>inchToTankTick(dist)){
                right.setPower(motorPower);
                left.setPower(motorPower);
                opMode.telemetry.addData("right current", rightCurPos-rightInitPos);
                opMode.telemetry.addData("goal", inchToTankTick(dist));
                opMode.telemetry.update();
                rightCurPos=right.getCurrentPosition();
            }
        }

        right.setPower(0);
        left.setPower(0);

    }

    // get current robot angle (radians)
    public double getIMUAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    public void spinToAngle(double angle) {
        double goalAngle = getIMUAngle() + angle;
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(angle - getIMUAngle()) > .15) {
            if (angle > 0) {
                //left.setPower(-0.5f);
                right.setPower(0.7f);
            } else {
                //right.setPower(-0.5f);
                left.setPower(0.7f);
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

