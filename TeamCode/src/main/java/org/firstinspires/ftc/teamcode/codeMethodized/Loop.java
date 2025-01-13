package org.firstinspires.ftc.teamcode.codeMethodized;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FFVar;

public class Loop {
    private Communicate communModule;
    private Calculate calcModule;

    public void handleLoop(Initialization initModule) {
        communModule.Talk(initModule);
        communModule.Listen(initModule);
        calcModule.Actions(initModule);
    }
}