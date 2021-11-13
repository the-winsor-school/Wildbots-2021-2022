package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TeleOp")
public class SimpleTeleOp extends LinearOpMode {

    private TankDrive tankDrive;

    //initializing

    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive = new TankDrive(this);
        telemetry.addData("status", "BAAAAAAAH initialized");
        telemetry.update();
        boolean alreadyPrinted =false;
        waitForStart();

        while(opModeIsActive() && !isStopRequested() ) {
            if(!alreadyPrinted){
                telemetry.addData("status", "OKAY WE'RE IN THE LOOP");
                alreadyPrinted=true;
            }
            tankDrive.Drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
