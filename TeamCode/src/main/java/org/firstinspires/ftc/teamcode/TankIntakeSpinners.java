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
@Disabled
@TeleOp(name = "tank spinner")
public class TankIntakeSpinners extends LinearOpMode {
    //private TankDrive tankDrive;
    public Orientation angles;
    private BNO055IMU imu; //gyroscope in rev hub
    float curHeading;

    public DcMotor duckSpinner;
    public DcMotor left;
    public DcMotor right;
    public DcMotor intakeSpinner;
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

        left = hardwareMap.tryGet(DcMotor.class, "left");
        right = hardwareMap.tryGet(DcMotor.class, "right");
        intakeSpinner = hardwareMap.tryGet(DcMotor.class, "intakeSpinner");

        boolean alreadyRun = false;

        //tankDrive = new TankDrive(this);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            this.imu.getPosition();
// and save the heading
            curHeading = angles.firstAngle;
            telemetry.addData("angle", curHeading);
            telemetry.update();
            if (!alreadyRun) {
                intakeSpinner.setPower(1);
                sleep(7000);
                intakeSpinner.setPower(0);
            }
        }
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


