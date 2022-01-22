package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.checkerframework.checker.units.qual.A;
import java.util.*;

@Autonomous(name = "Test FSR aka TaxEvasion")
public class FSRTest extends LinearOpMode { //Force Sensitive Resistor (FSR)
    AnalogInput forceSensitiveResistor;
    double currentForce;

    @Override
    public void runOpMode() {
        forceSensitiveResistor = hardwareMap.get(AnalogInput.class, "Force Sensitive Resistor");

        waitForStart();
        while(opModeIsActive()){
            currentForce = forceSensitiveResistor.getVoltage();

            if (currentForce > 0.113 && currentForce < 0.169) {
                telemetry.addData("Box Weight:", "Light");
                telemetry.update();
            } else if (currentForce > 0.189 && currentForce < 0.224) {
                telemetry.addData("Box Weight:", "Medium");
                telemetry.update();
            } else if (currentForce > 0.235 && currentForce < 0.278){
                telemetry.addData("Box Weight:", "Heavy");
                telemetry.update();
            } else {
                telemetry.addData("Force", currentForce);
                telemetry.update();
            }
        }
    }
}