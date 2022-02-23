package org.firstinspires.ftc.teamcode;

public class Skills extends Bot{

    public void Move(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void Turn(){

    }

    public void Arm(){

    }

    public void Claw(){

    }

    public void Spin (float power){

    }

    public void Dozer (){

    }

    public float CalculateSpeed (float joyX, float joyY){

        return joyX + joyY;
    }

    public boolean isJoyActive (String joyToCheck){

        return true;
    }
}