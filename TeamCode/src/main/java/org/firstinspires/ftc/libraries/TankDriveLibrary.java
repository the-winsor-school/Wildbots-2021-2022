package org.firstinspires.ftc.libraries;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TankDriveLibrary {
    // hardware variables
    public DcMotor left;
    public DcMotor right;

    private DcMotor[] allMotors;
    private HardwareMap hardwareMap;
    private OpMode opMode;

    // sensor variables
    private BNO055IMU imu; //gyroscope in rev hub
    private Orientation angles;
    private Acceleration gravity;

    private double targetAngle;

    public TankDriveLibrary(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;

        left = hardwareMap.tryGet(DcMotor.class, "left");
        right = hardwareMap.tryGet(DcMotor.class, "right");

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

        targetAngle = getIMUAngle();
    }

    public double getIMUAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    public void Drive(double l, double r) {
        left.setPower(l);
        right.setPower(-r);
    }

    public void Forward () {
        left.setPower(1);
        right.setPower(-1);
    }

    public void Backward () {
        left.setPower(-1);
        right.setPower(1);
    }

    public void spinRight () {
        left.setPower(1);
        right.setPower(1);
    }

    public void spinLeft () {
        left.setPower(-1);
        right.setPower(-1);
    }

    public void stopDriving () {
        left.setPower(0);
        right.setPower(0);
    }

    public void spinToAngle(double angle) {
        double goalAngle = getIMUAngle() + angle;
        while (Math.abs(angle - getIMUAngle()) > .15) {
            if (angle > 0) {
                spinRight();
            }
            else {
                spinLeft();
            }
            opMode.telemetry.update();
        }
        stopDriving();
        targetAngle = getIMUAngle();
    }
}