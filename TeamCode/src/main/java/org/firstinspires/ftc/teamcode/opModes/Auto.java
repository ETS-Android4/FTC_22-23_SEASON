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
    private Servo sherry;
    private DcMotorEx sheral;



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
        sherry = hardwareMap.get(Servo.class, "claw_servo");
        sheral = hardwareMap.get(DcMotorEx.class, "spin_motor");




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

        //code goes here ------------------------

        TurnPlacesNew("LEFTBACK", .6, 2000);


        //code goes here ------------------------




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("encoder value", 4);
            telemetry.update();
        }
    }

    private void DrivePlaces (String direction, double speed, int distance)
    {
        direction = direction.toUpperCase();

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
            default:
                dylan.setTargetPosition(0);
                jerry.setTargetPosition(0);
                bob.setTargetPosition(0);
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

    private void TurnPlacesNew (String direction, double speed, int mSecs)
    {

        dylan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bob.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        larry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        direction = direction.toUpperCase();

        if (direction == "RIGHTFRONT")
        {
            dylan.setPower(0);
            jerry.setPower(0);
            bob.setPower(speed);
            larry.setPower(speed);
        }
        else if (direction == "RIGHTBACK")
        {
            dylan.setPower(0);
            jerry.setPower(0);
            bob.setPower(-speed);
            larry.setPower(-speed);
        }
        else if (direction == "LEFTFRONT")
        {
            dylan.setPower(speed);
            jerry.setPower(speed);
            bob.setPower(0);
            larry.setPower(0);
        }
        else if (direction == "LEFTBACK")
        {
            dylan.setPower(-speed);
            jerry.setPower(-speed);
            bob.setPower(0);
            larry.setPower(0);
        }

        sleep(mSecs);

        dylan.setPower(0);
        jerry.setPower(0);
        bob.setPower(0);
        larry.setPower(0);
    }

    public void Arm(ArmPositions position, float power) {
        switch (position) {
            //set arm position and wrist position
            case START:
                barry.setTargetPosition(0);
                break;
            case PICKUP:
                barry.setTargetPosition(2);
                garry.setPosition(0.3069);
                break;
            case BOTTOM:
                barry.setTargetPosition(300);
                break;
            case MIDDLE:
                barry.setTargetPosition(500);
                break;
            case TOP:
                barry.setTargetPosition(1400);
                break;
            case TOPTOP:
                barry.setTargetPosition(1700);
                break;


        }
        barry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        barry.setPower(power);
    }

    public void Claw(float position) {
        sherry.setPosition(position);
    }

    public void spin(int mSecs){
        sheral.setPower(0);
        sleep(mSecs);
    }
}

