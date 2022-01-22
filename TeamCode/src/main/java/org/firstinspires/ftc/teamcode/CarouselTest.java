//adb connect 192.168.43.1:5555

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;


@TeleOp(name  = "Carousel Test", group = "Finished")
public class CarouselTest extends LinearOpMode {
    DcMotor carousel;

    public void runOpMode() throws InterruptedException {
        carousel = hardwareMap.get(DcMotor.class, "carousel");;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            carousel.setPower(gamepad2.right_stick_y); //based on positioning of the joystick, motor will spin

            if(gamepad2.a) {
                carousel.setPower((carousel.getPower() == -1) ? 0 : 1);
            }

            if(gamepad2.x) {
                carousel.setPower(-1);
                sleep(3500);
                carousel.setPower(0);
            }

            telemetry.addData("Status", "Running"); //prints to phone
            telemetry.addData("Power: ", carousel.getPower());

            telemetry.update(); //makes actually print
        }
    }
}