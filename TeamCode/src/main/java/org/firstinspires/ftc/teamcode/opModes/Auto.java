package org.firstinspires.ftc.teamcode.opModes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.res.ArmPositions;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoBoi", group="opModes")

public class Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

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

        dylan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        larry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        barry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dylan.setDirection((DcMotorSimple.Direction.FORWARD));
        jerry.setDirection(DcMotorSimple.Direction.FORWARD);
        bob.setDirection(DcMotorSimple.Direction.REVERSE);
        larry.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //test encoders on motor
        DrivePlaces("RIGHT", .8, 5000);
        DrivePlaces("Forward", .8, 2000);
        DrivePlaces("FORWARD/RIGHT", .8, 6000);
        DrivePlaces("BACKWARD", .8, 2000);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("encoder value", jerry.getCurrentPosition());
            telemetry.update();
        }
    }

    private void DrivePlaces (String direction, double speed, int distance)
    {
        distance = abs(distance);
        switch (direction)
        {
            case "STOP":
                // Stop
                dylan.setPower(0);
                jerry.setPower(0);
                bob.setPower(0);
                larry.setPower(0);
                break;
            case "FORWARD":
                // Drive forward
                dylan.setTargetPosition(distance);
                jerry.setTargetPosition(distance);
                bob.setTargetPosition(distance);
                larry.setTargetPosition(distance);
                break;
            case "FORWARD/RIGHT":
                // Drive forward/right
                dylan.setTargetPosition(distance);
                jerry.setTargetPosition(0);
                bob.setTargetPosition(0);
                larry.setTargetPosition(distance);
                break;
            case "RIGHT":
                // Drive right
                dylan.setTargetPosition(distance);
                jerry.setTargetPosition(-distance);
                bob.setTargetPosition(-distance);
                larry.setTargetPosition(distance);
                break;
            case "BACKWARD/RIGHT":
                // Drive backward/right
                dylan.setTargetPosition(0);
                jerry.setTargetPosition(-distance);
                bob.setTargetPosition(-distance);
                larry.setTargetPosition(0);
                break;
            case "BACKWARD":
                // Drive backward
                dylan.setTargetPosition(-distance);
                jerry.setTargetPosition(-distance);
                bob.setTargetPosition(-distance);
                larry.setTargetPosition(-distance);
                break;
            case "BACKWARD/LEFT":
                // Drive backward/left
                dylan.setTargetPosition(-distance);
                jerry.setTargetPosition(0);
                bob.setTargetPosition(0);
                larry.setTargetPosition(-distance);
                break;
            case "LEFT":
                // Drive left
                dylan.setTargetPosition(-distance);
                jerry.setTargetPosition(distance);
                bob.setTargetPosition(distance);
                larry.setTargetPosition(-distance);
                break;
            case "FORWARD/LEFT":
                // Drive forward/left
                dylan.setTargetPosition(0);
                jerry.setTargetPosition(distance);
                bob.setTargetPosition(distance);
                larry.setTargetPosition(0);
                break;
        }
        dylan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jerry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        larry.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dylan.setPower(speed);
        jerry.setPower(speed);
        bob.setPower(speed);
        larry.setPower(speed);

        while (opModeIsActive() && (dylan.isBusy()|| jerry.isBusy()|| bob.isBusy()|| larry.isBusy()))
        {
            telemetry.addData("encoder-fwd-left", dylan.getCurrentPosition() + "  busy=" + dylan.isBusy());
            telemetry.addData("encoder-fwd-right", dylan.getCurrentPosition() + "  busy=" + dylan.isBusy());
            telemetry.update();
            idle();
        }
        dylan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        larry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void TurnPlacesNew (double Joy_x, double Joy_y, double speed)
    {
        if (Joy_x > 0 && Joy_y > 0)
        {
            dylan.setPower(0);
            jerry.setPower(0);
            bob.setPower(speed);
            larry.setPower(speed);
        }
        else if (Joy_x > 0 && Joy_y < 0)
        {
            dylan.setPower(0);
            jerry.setPower(0);
            bob.setPower(-speed);
            larry.setPower(-speed);
        }
        else if (Joy_x < 0 && Joy_y > 0)
        {
            dylan.setPower(speed);
            jerry.setPower(speed);
            bob.setPower(0);
            larry.setPower(0);
        }
        else if (Joy_x < 0 && Joy_y < 0)
        {
            dylan.setPower(-speed);
            jerry.setPower(-speed);
            bob.setPower(0);
            larry.setPower(0);
        }
    }


    private void Arm(ArmPositions positionToMove) {

    }
    public void ClawFunction()
    {
        garry.setPosition(0);
    }
}

