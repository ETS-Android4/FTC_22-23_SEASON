package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class BotDebugger extends Bot{

    private HardwareMap mappings = null;

    public BotDebugger(HardwareMap hwMap)
    {
        super.initializeHWMap(hwMap);
        mappings = hwMap;
    }

    public CurrentUnit PrintPowerDraw (ArrayList<HardwareDevice> devices)
    {
        for (HardwareDevice d : devices)
        {

        }

        CurrentUnit total_power_draw = CurrentUnit.AMPS;

        return total_power_draw;
    }

    public void main(String args)
    {

    }
}
