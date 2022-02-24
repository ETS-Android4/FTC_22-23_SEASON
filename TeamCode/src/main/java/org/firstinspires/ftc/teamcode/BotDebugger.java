package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class BotDebugger extends Bot{

    public double PrintPowerDraw () {
        double power_draw;

        power_draw = ((frontLeftMotor.getCurrent(CurrentUnit.MILLIAMPS) + frontRightMotor.getCurrent(CurrentUnit.MILLIAMPS)
                + backLeftMotor.getCurrent(CurrentUnit.MILLIAMPS) + backRightMotor.getCurrent(CurrentUnit.MILLIAMPS)) / 4);

        return power_draw;
    }
}
