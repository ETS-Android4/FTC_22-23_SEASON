/*#################### IMPORTS ####################*/
package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
/*#################### END IMPORTS ####################*/

@TeleOp(name = "GudCode", group = "opModes") //Name and type declaration for the drivers station
public class DriverControlled extends LinearOpMode {
    /*####################  VARIABLE DECLARATIONS ####################*/

    // Chassis motors
    private DcMotorEx bob; // Front-left chassis motor
    private DcMotorEx dylan; // Front-right chassis motor
    private DcMotorEx larry; // Back-left chassis motor
    private DcMotorEx jerry; // Back-right chassis motor
    private DcMotorEx sheral; //Spinner


    // Arm motors/servos
    private DcMotorEx barry;
    private Servo garry;
    private Servo sherry;
    private Servo bulldozer;

    //imu garbage
    private BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //end imu garbage

    private double clawCurrentPosition = 0.5; // Servos default to 0.5 as not moving

    double wrist_position = 0;
    double wrist_offset = 0; // Allows for manual control of wrist angle
    boolean was_pressed = true; // Flag used for detecting when to hold the arm in position
    boolean bulldozer_flag = true; //Flag used for bulldozer to toggle
    boolean turnForward = true; //Flag to see if robot turns forward or backward
    boolean anotherFlag = true; //second flag for toggling

    double speed_limiter = 0.4;

    /*private static final String VUFORIA_KEY =
            "AbskhHb/////AAABmb8nKWBiYUJ9oEFmxQL9H2kC6M9FzPa1acXUaS/H5wRkeNbpNVBJjDfcrhlTV2SIGc/lxBOtq9X7doE2acyeVOPg4sP69PQQmDVQH5h62IwL8x7BS/udilLU7MyX3KEoaFN+eR1o4FKBspsYrIXA/Oth+TUyrXuAcc6bKSSblICUpDXCeUbj17KrhghgcgxU6wzl84lCDoz6IJ9egO+CG4HlsBhC/YAo0zzi82/BIUMjBLgFMc63fc6eGTGiqjCfrQPtRWHdj2sXHtsjZr9/BpLDvFwFK36vSYkRoSZCZ38Fr+g3nkdep25+oEsmx30IkTYvQVMFZKpK3WWMYUWjWgEzOSvhh+3BOg+3UoxBJSNk";
    */
    /*#################### END VARIABLE DECLARATIONS ####################*/

    @Override
    public void runOpMode() {
        /*#################### INITALIZATION STAGE ####################*/
        // Map variables to motors in driver station configuration
        bob = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        dylan = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        larry = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        jerry = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        barry = hardwareMap.get(DcMotorEx.class, "swing_arm_motor");
        sheral = hardwareMap.get(DcMotorEx.class, "spin_motor");
        garry = hardwareMap.get(Servo.class, "wrist_joint");
        sherry = hardwareMap.get(Servo.class, "claw_servo");
        bulldozer = hardwareMap.get(Servo.class, "bulldozer");


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Setting motor direction internally so i dont have to do -(-(-(-speed)))
        dylan.setDirection((DcMotorSimple.Direction.REVERSE));
        jerry.setDirection(DcMotorSimple.Direction.FORWARD);
        bob.setDirection(DcMotorSimple.Direction.FORWARD);
        larry.setDirection(DcMotorSimple.Direction.REVERSE);
        sheral.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resetting motor encoders
        dylan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        larry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sheral.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        barry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Returning motors to regular runmode
        dylan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jerry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bob.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        larry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sheral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart(); // Activated upon pressing play (after pressing initalize)

        /*#################### END INITALIZATION STAGE ####################*/

        /*#################### RUNNING STAGE ####################*/

        // Main while loop (this runs until stopped from the drivers station)
        while (opModeIsActive()) {

            // Placeholders to make the math easier
            double rx = this.gamepad1.right_stick_x;
            double ry = -this.gamepad1.right_stick_y;
            double lx = this.gamepad1.left_stick_x;
            double ly = -this.gamepad1.left_stick_y;

            // Calculate point on the circumference of the circle to use as the joystick location (distance formula)
            double rightJoy_x = rx / Math.sqrt(((Math.pow(rx, 2)) + (Math.pow(ry, 2))));
            double rightJoy_y = ry / Math.sqrt(((Math.pow(rx, 2)) + (Math.pow(ry, 2))));
            double leftJoy_x = lx / Math.sqrt(((Math.pow(lx, 2)) + (Math.pow(ly, 2))));
            double leftJoy_y = ly / Math.sqrt(((Math.pow(lx, 2)) + (Math.pow(ly, 2))));

            // Logic to give determine which stick is in use (gives priority to the left stick when both are in use)
            if (JoyIsActive("left")) {
                TurnPlacesNew(leftJoy_x, calculatedSpeed("left"));
            } else if (JoyIsActive("right")) {
                DrivePlaces(calculatedDirection(rightJoy_x, rightJoy_y), .8*calculatedSpeed("right"));
            } else {
                dylan.setPower(0);
                jerry.setPower(0);
                bob.setPower(0);
                larry.setPower(0);
            }

            if (this.gamepad1.y) {ReInit();} // Y resets the encoders in all chassis motors

            ToggleBulldozer();
            // Collectivly causes arm movement
            MoveClaw();
            MoveArm();

            // Spins the spinner for the ducks
            if (this.gamepad1.a) {sheral.setPower(0.1);}
            else {sheral.setPower(0);}

            // Complex shutdown condition so we don't accidentally turn the robot off
            if (this.gamepad1.left_trigger == 1 && this.gamepad1.right_trigger == 1 && this.gamepad1.left_bumper && this.gamepad1.right_bumper)
            {
                DriverShutdown();
            }

            //
            if (this.gamepad1.b && anotherFlag){
                if (speed_limiter == 0.4) {
                    speed_limiter = 1;
                }
                else {
                    speed_limiter = 0.4;
                }
                anotherFlag = false;
            }else if (!this.gamepad1.b && !anotherFlag) {
                anotherFlag = true;
            }

            /*#################### TROUBLESHOOTING ####################*/
            telemetry.addData("Right stick X", rightJoy_x);
            telemetry.addData("Right stick Y", rightJoy_y);

            telemetry.addData("Left stick X", leftJoy_x);
            telemetry.addData("Left stick Y", leftJoy_y);

            telemetry.addData("Arm Position: ", barry.getCurrentPosition());
            telemetry.addData("Wrist Position: ", garry.getPosition());

            telemetry.update();
            /*#################### TROUBLESHOOTING ####################*/
        }
    }
    /*#################### END RUNNING STAGE ####################*/

    /*#################### FUNCTIONS ####################*/
    // Drives in the provided direction at the provided power
    private void DrivePlaces (String directionInput, double speed)
    {
        String direction = directionInput.toUpperCase(); // Prevents capitalization errors because that causes pain

        switch (direction) //Determines which direction to travel based on input
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
                dylan.setPower(speed*speed_limiter);
                jerry.setPower(speed*speed_limiter);
                bob.setPower(speed*speed_limiter);
                larry.setPower(speed*speed_limiter);
                break;
            case "FORWARD/RIGHT":
                // Drive forward/right
                dylan.setPower(speed*speed_limiter);
                jerry.setPower(0);
                bob.setPower(0);
                larry.setPower(speed*speed_limiter);
                break;
            case "RIGHT":
                // Drive right
                dylan.setPower(speed*speed_limiter);
                jerry.setPower(-speed*speed_limiter);
                bob.setPower(-speed*speed_limiter);
                larry.setPower(speed*speed_limiter);
                break;
            case "BACKWARD/RIGHT":
                // Drive backward/right
                dylan.setPower(0);
                jerry.setPower(-speed*speed_limiter);
                bob.setPower(-speed*speed_limiter);
                larry.setPower(0);
                break;
            case "BACKWARD":
                // Drive backward
                dylan.setPower(-speed*speed_limiter);
                jerry.setPower(-speed*speed_limiter);
                bob.setPower(-speed*speed_limiter);
                larry.setPower(-speed*speed_limiter);
                break;
            case "BACKWARD/LEFT":
                // Drive backward/left
                dylan.setPower(-speed*speed_limiter);
                jerry.setPower(0);
                bob.setPower(0);
                larry.setPower(-speed*speed_limiter);
                break;
            case "LEFT":
                // Drive left
                dylan.setPower(-speed*speed_limiter);
                jerry.setPower(speed*speed_limiter);
                bob.setPower(speed*speed_limiter);
                larry.setPower(-speed*speed_limiter);
                break;
            case "FORWARD/LEFT":
                // Drive forward/left
                dylan.setPower(0);
                jerry.setPower(speed*speed_limiter);
                bob.setPower(speed*speed_limiter);
                larry.setPower(0);
                break;
        }
    }

    // Turns by leaving one side off and turning the other side on
    private void TurnPlacesNew (double Joy_x, double speed)
    {
        if( Joy_x > 0){
            dylan.setPower(-speed*speed_limiter);
            jerry.setPower(-speed*speed_limiter);
            bob.setPower(speed*speed_limiter);
            larry.setPower(speed*speed_limiter);
        }else if (Joy_x<0){
            dylan.setPower(speed*speed_limiter);
            jerry.setPower(speed*speed_limiter);
            bob.setPower(-speed*speed_limiter);
            larry.setPower(-speed*speed_limiter);
        }
    }

    // Returns a direction based on joystick inputs (WARNING: Uses cosine stuff so just trust Mr. Beckman on this one)
    private String calculatedDirection (double Joy_x, double Joy_y)
    {
        String directionToTravel;

        // Divides the joystick into 8 possible slices and determines a direction based on which slice it's in
        if (Joy_x > Math.cos(1.96) && Joy_x < Math.cos(1.18) && Joy_y > 0)
        {
            directionToTravel = "FORWARD"; // Forwards direction
        }
        else if (Joy_x > Math.cos(1.18) && Joy_x < Math.cos(0.393)  && Joy_y > 0)
        {
            //directionToTravel = "FORWARD/RIGHT"; // Forward/right direction
            directionToTravel = "FORWARD";
        }
        else if (Joy_x > 0 && Joy_y > Math.sin(-0.393) && Joy_y < Math.sin(0.393))
        {
            directionToTravel = "RIGHT"; // Right direction
        }
        else if (Joy_x > Math.cos(1.18) && Joy_x < Math.cos(0.393)  && Joy_y < 0)
        {
            //directionToTravel = "BACKWARD/RIGHT"; // Backward/right direction
            directionToTravel = "BACKWARD"; //
        }
        else if (Joy_x > Math.cos(1.96) && Joy_x < Math.cos(1.18) && Joy_y < 0)
        {
            directionToTravel = "BACKWARD"; // Backwards direction
        }
        else if (Joy_x < Math.cos(1.96) && Joy_x > Math.cos(2.75) && Joy_y < 0)
        {
            //directionToTravel = "BACKWARD/LEFT"; // Backward/left direction
            directionToTravel = "BACKWARD";
        }
        else if (Joy_x < 0 && Joy_y > Math.sin(-0.393) && Joy_y < Math.sin(0.393))
        {
            directionToTravel = "LEFT"; // Left direction
        }
        else if (Joy_x < Math.cos(1.96) && Joy_x > Math.cos(2.75) && Joy_y > 0)
        {
            //directionToTravel = "FORWARD/LEFT"; // Forward/left direction
            directionToTravel = "FORWARD/RIGHT";
        }
        else
        {
            directionToTravel = "STOP"; // Stop just as a precaution
        }

        return directionToTravel;
    }

    // Calculates the magnitude on the joysticks position and returns it as a power value for the motors
    private double calculatedSpeed (String joyStickInput)
    {
        String joyStick = joyStickInput.toLowerCase(); // Prevents capitalization errors

        double speed;

        // Repeat of initial values, but only applies to this method (it's bad practice but fixed the problem so idk)
        double rx = this.gamepad1.right_stick_x;
        double ry = -this.gamepad1.right_stick_y;
        double lx = this.gamepad1.left_stick_x;
        double ly = -this.gamepad1.left_stick_y;

        // Calculates magnitude of the current stick
        if (joyStick.equals("right"))
        {
            speed = Math.sqrt(Math.pow(rx, 2) + Math.pow(ry, 2));
        }
        else if(joyStick.equals("left"))
        {
            speed = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));
        }
        else {speed = 0;}

        // Maximum percentage of power the motors are allowed to use
        double speed_cap = 0.75;
        return speed * speed_cap; // Applies a limiter on the speed
    }

    // Returns weither the selected joystick is in use (has a value larger than the deadzone)
    private boolean JoyIsActive (String joyStickInput)
    {
        boolean isActive = false;
        double rx = this.gamepad1.right_stick_x;
        double ry = -this.gamepad1.right_stick_y;
        double lx = this.gamepad1.left_stick_x;
        double ly = -this.gamepad1.left_stick_y;

        String joyStick = joyStickInput.toLowerCase(); // Prevents capitalization errors

        // Distance required to trigger detection of the joysticks
        double deadzone = 0.01;
        if (joyStick.equals("right"))
        {
            isActive = Math.abs(rx) > deadzone || Math.abs(ry) > deadzone;
        }
        else if (joyStick.equals("left"))
        {
            isActive = Math.abs(lx) > deadzone || Math.abs(ly) > deadzone;
        }

        return isActive;
    }

    // Responsible for wrist movement and the opening/closing of the claw
    private void MoveClaw()
    {
        // Incriment the desired position while holding the corresponding direction on the dpad
        if (this.gamepad1.dpad_left || this.gamepad1.dpad_right) {
            if (this.gamepad1.dpad_left) // Open
            {
                if (clawCurrentPosition > 0.75) {
                    clawCurrentPosition = 0.75;
                } else {
                    clawCurrentPosition += 0.05;
                }
            } else if (this.gamepad1.dpad_right) // Close
            {
                if (clawCurrentPosition < 0.1) {
                    clawCurrentPosition = 0.1;
                } else {
                    clawCurrentPosition -= 0.05;
                }
            }
        }
        else {
            wrist_position = barry.getCurrentPosition() / 1700.0; // Keeps the claw level when moving the arm

            // Incriment the offset of the wrist (up or down) with the dpad (applies when leveling with the arm)
            if (this.gamepad1.dpad_down && (wrist_position + wrist_offset <= 1)) {
                wrist_offset += 0.005;
            } else if (this.gamepad1.dpad_up && (wrist_position + wrist_offset >= 0)) {
                wrist_offset -= 0.005;
            }
        }

        // Move the wrist to allow for picking up cubes/balls (ignores offset to prevent damage)
        /*if (barry.getCurrentPosition() < 100) {garry.setPosition(0.4069);}

        else {garry.setPosition(wrist_position + wrist_offset);} // Actually setting the position of wrist
        */
        garry.setPosition(wrist_position + wrist_offset);

        sherry.setPosition(clawCurrentPosition); // Actually open/close the claw
    }

    // Responsible for arm movement
    private void MoveArm()
    {
        // Moves arm when triggers are pressed (speed determined by how far the trigger is pressed)
        // Holds the arm in it's current position when it's not moving
        if (this.gamepad1.right_trigger > 0) {
            barry.setTargetPosition(1700);
            barry.setPower(this.gamepad1.right_trigger);
            was_pressed = true;
        } else if (this.gamepad1.left_trigger > 0) {
            barry.setTargetPosition(0);
            barry.setPower(this.gamepad1.left_trigger / 2);
            was_pressed = true;
        } else if (was_pressed) {
            barry.setTargetPosition(barry.getCurrentPosition());
            barry.setPower(1);
            was_pressed = false;
        }

        barry.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Actually move the arm
    }

    // Resets motor encoders (in case everything is screwed up)
    private void ReInit()
    {
        bob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dylan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        larry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jerry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        barry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(25);

        bob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dylan.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        larry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jerry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void DriverShutdown()
    {
        // Stop all chassis motors
        dylan.setPower(0);
        jerry.setPower(0);
        bob.setPower(0);
        larry.setPower(0);

        //  Stop the spinner
        sheral.setPower(0);

        // Return arm and claw to starter position
        sherry.setPosition(0.5);
        garry.setPosition(0.3069);
        barry.setTargetPosition(0);
        barry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        barry.setPower(1);

        sleep(1000); // Gives time for the motors and servos to return to starter position before shutdown

        requestOpModeStop(); // Attempt to end the opmode
    }

    private void ToggleBulldozer(){

        if (this.gamepad1.x && bulldozer_flag) {
            if(bulldozer.getPosition()==.1){
                bulldozer.setPosition(.8);
            }else{
                bulldozer.setPosition(.1);
            }
            bulldozer_flag = false;
        }else if (!this.gamepad1.x && !bulldozer_flag){
            bulldozer_flag = true;
        }


    }

    /*#################### END FUNCTIONS ####################*/
}
