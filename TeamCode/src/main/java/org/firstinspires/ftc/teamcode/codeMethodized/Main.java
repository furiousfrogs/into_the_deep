package org.firstinspires.ftc.teamcode.codeMethodized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main",group="TeleOp")
public class Main extends OpMode {

    private Initialization initModule;
    private Loop loopModule;
    private drive driveModule;
    private telemetrics teleModule;

    @Override
    public void init() {
        initModule = new Initialization();
        loopModule = new Loop();
        driveModule= new drive();
        teleModule=new telemetrics();
        initModule.initialize(hardwareMap, gamepad1);
    }

    @Override
    public void loop() {
        loopModule.handleLoop(initModule);
        driveModule.driver(initModule);
        teleModule.telem(initModule);
    }
}