package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class TeleOPSkills extends Bot{

    public TeleOPSkills(HardwareMap hwMap) {super.initializeHWMap(hwMap);}

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

    public void ChangeArmPosition(int positionChange, double power)
    {
        armMotor.setTargetPosition(Math.min(armMotor.getCurrentPosition() + positionChange, 1700));

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    public void ChangeClawPosition(double positionChange)
    {
        if (clawServo.getPosition() + positionChange > 1) {clawServo.setPosition(1);}
        else if (clawServo.getPosition() + positionChange < 0) {clawServo.setPosition(0);}
        else {clawServo.setPosition(clawServo.getPosition() + positionChange);}
    }

    public void ChangeWristOffset(double positionChange) {
        if (wristServo.getPosition() + positionChange > 1) {wristServo.setPosition(1);}
        else if (wristServo.getPosition() + positionChange < 0) {wristServo.setPosition(0);}
        else {wristServo.setPosition(wristServo.getPosition() + positionChange);}
    }

    public void Spin (float power){
        spinMotor.setPower(power);
    }
}