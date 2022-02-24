package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Skills extends Bot{

    public Skills(HardwareMap hwMap) {super.initializeHWMap(hwMap);}

    public void Move(double axial, double lateral, double yaw, double speed_limiter){
        double max;

        // Forward returns negative value
        axial = -axial;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftPower  *= speed_limiter;
        frontRightPower *= speed_limiter;
        backLeftPower   *= speed_limiter;
        backRightPower  *= speed_limiter;

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void MoveArm(int targetHeight, double power)
    {
        if (targetHeight > 1700) {
            targetHeight = 1700;
        }

        armMotor.setTargetPosition(targetHeight);
        clawServo.setPosition(armMotor.getTargetPosition()/1700);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    public void Claw(double claw_position) {
        clawServo.setPosition(claw_position);
    }

    public void ChangeWristOffset(double wrist_position) {
        wristServo.setPosition(wristServo.getPosition() + wrist_position);
    }

    public void Spin (float power){
        spinMotor.setPower(power);
    }

    public void Dozer (){

    }

    public float CalculateSpeed (String joy){

        return 1;
    }

    public boolean JoyIsActive (String joyToCheck){

        return true;
    }
}