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

            telemetry.addData("Force", currentForce);
            telemetry.update();
        }
    }
}
