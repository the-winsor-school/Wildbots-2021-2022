package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
@TeleOp(name = "reset rotini but it doesn't work yet")
public class resetRotini extends LinearOpMode {
    //private TankDrive tankDrive;
    public Orientation angles;
    private BNO055IMU imu; //gyroscope in rev hub
    float curHeading;

    public DcMotor duckSpinner;
    public DcMotor rotini;
    public DcMotor left;
    public DcMotor right;

    int initPos;
    int curPos;
    //private int encoderValues = 0;
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        rotini=hardwareMap.tryGet(DcMotor.class, "rotini");
        left = hardwareMap.tryGet(DcMotor.class, "left");
        right = hardwareMap.tryGet(DcMotor.class, "right");

        boolean alreadyRun = false;
        boolean firstTime=true;

        //tankDrive = new TankDrive(this);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (firstTime){
                initPos=rotini.getCurrentPosition();
                curPos=rotini.getCurrentPosition();
                    rotini.setPower(0.5);
                    sleep(400);
                rotini.setPower(0);
                rotini.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                firstTime=false;
            }
        }
    }
    public int inToTick(int inches){
        return inches*40;
    }
    public void spinToAngle(double angle){
        double goalAngle = getIMUAngle() + angle;
        while (Math.abs(angle - getIMUAngle()) > .15) {
            if (angle > 0) {
                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left.setPower(-0.5f);
            } else {
                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right.setPower(0.5f);
            }
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
        //targetAngle = getIMUAngle();
    }

    public double getIMUAngle () {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        return angle;
    }
// read the orientation of the robot
}


