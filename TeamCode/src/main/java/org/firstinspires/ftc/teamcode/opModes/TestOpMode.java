package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOPSkills;

@TeleOp(name = "FirstOPMode", group = "OpModes")
public class TestOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Logic Flags
    private boolean arm_flag = true;
    private boolean spinner_flag = true;

    @Override
    public void runOpMode() throws InterruptedException {

        TeleOPSkills skills = new TeleOPSkills(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            skills.Move(this.gamepad1.left_stick_y, this.gamepad1.left_stick_x, this.gamepad1.right_stick_x, 0.5);

            if (this.gamepad1.right_trigger > 0) {
                skills.MoveArm(1700, this.gamepad1.right_trigger);
                arm_flag = true;
            } else if (this.gamepad1.left_trigger > 0) {
                skills.MoveArm(0, this.gamepad1.right_trigger);
                arm_flag = true;
            } else if (arm_flag) {
                skills.MoveArm(skills.armMotor.getTargetPosition(), 0.25);
                arm_flag = false;
            }

            if (this.gamepad1.dpad_left || this.gamepad1.dpad_right) {
                if (this.gamepad1.dpad_left) {
                    skills.Claw(1);
                }
                else if (this.gamepad1.dpad_right) {
                    skills.Claw(0);
                }
            }
            else {
                if (this.gamepad1.dpad_up) {
                    skills.ChangeWristOffset(0.05);
                }
                else if (this.gamepad1.dpad_down) {
                    skills.ChangeWristOffset(-0.05);
                }
            }

            if (this.gamepad1.a && spinner_flag)
            {
                if (skills.spinMotor.getPower() == 1) {
                    skills.Spin(0);
                }
                else if (skills.spinMotor.getPower() == 0) {
                    skills.Spin(1);
                }
                spinner_flag = false;
            }
            else if (!this.gamepad1.a && !spinner_flag)
            {
                spinner_flag = true;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
