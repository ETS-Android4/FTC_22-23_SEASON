package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Skills extends Bot{

    public static void Move(){

    }

    public static void Turn(){

    }

    public static void Arm(){

    }

    public static void Claw(){

    }

    public static void Spin (){

    }

    public static void Dozer (){

    }

    public static float CalculateSpeed (float joyX, float joyY){

        return joyX + joyY;
    }

    public static boolean isJoyActive (String joyToCheck){

        return true;
    }
}