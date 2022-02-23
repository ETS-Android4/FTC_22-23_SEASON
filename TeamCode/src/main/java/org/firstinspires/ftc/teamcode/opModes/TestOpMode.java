package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Skills;
import org.firstinspires.ftc.teamcode.Bot;

@Autonomous(name = "TestNewStructure", group = "OpModes")
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Skills skills = new Skills(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            skills.spinMotor.setPower(1);
        }
    }
}
