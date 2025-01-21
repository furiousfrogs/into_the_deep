package org.firstinspires.ftc.teamcode.codeMethodized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main",group="TeleOp")
public class Main extends OpMode {

    private Initialization initModule;
    private Loop loopModule;

    @Override
    public void init() {
        initModule = new Initialization();
        loopModule = new Loop();
        initModule.initialize(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        loopModule.handleLoop(initModule);
    }
}