package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankDrive {
    // hardware variables
    public DcMotor left;
    public DcMotor right;

    private DcMotor[] allMotors;
    private HardwareMap hardwareMap;
    private OpMode opMode;

    public TankDrive(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;

        left = hardwareMap.tryGet(DcMotor.class, "left");
        right = hardwareMap.tryGet(DcMotor.class, "right");
    }

    public void Drive(double l, double r) {
        left.setPower(l);
        right.setPower(-r);
    }
}