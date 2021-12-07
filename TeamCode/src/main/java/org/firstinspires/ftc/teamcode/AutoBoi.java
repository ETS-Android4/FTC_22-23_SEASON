package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class AutoBoi extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private ClawPositions position = ClawPositions.START;

    // Chassis motors
    private DcMotorEx bob;
    private DcMotorEx dylan;
    private DcMotorEx larry;
    private DcMotorEx jerry;

    // Arm motors/servos
    private DcMotorEx barry;
    private Servo garry;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Hardware mapping
        bob = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        dylan = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        larry = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        jerry = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        barry = hardwareMap.get(DcMotorEx.class, "swing_arm_motor");
        garry = hardwareMap.get(Servo.class, "wrist_joint");

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        dylan.setDirection((DcMotorSimple.Direction.REVERSE));
        jerry.setDirection(DcMotorSimple.Direction.REVERSE);
        bob.setDirection(DcMotorSimple.Direction.FORWARD);
        larry.setDirection(DcMotorSimple.Direction.FORWARD);

        barry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        Drive(.7f);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData()
            telemetry.update();
        }
    }

    public void Drive(float powa){
        dylan.setPower(powa);
        jerry.setPower(powa);
        bob.setPower(powa);
        larry.setPower(powa);
    }
    private void Arm(ClawPositions positionToMove) {
        switch (positionToMove) {
            case START:
                garry.setPosition(0);
                break;
            case PICKUP:
                garry.setPosition(0); //Something thats not 0
                break;
            case BOTTOM:
                garry.setPosition(0); //Something thats not 0
                break;
            case MIDDLE:
                garry.setPosition(0); //Something thats not 0
                break;
            case TOP:
                garry.setPosition(0); //Something thats not 0
                break;
            case TOPTOP:
                garry.setPosition(0); //Something thats not 0
                break;
        }
    }
    public void ClawFunction()
    {
        garry.setPosition(0);
    }

}

