package org.firstinspires.ftc.teamcode.codeMethodized;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main",group="TeleOp")
public class Main extends OpMode {

    private Initialization initModule;
    private Loop loopModule;

    @Override
    public void init() {
        initModule = new Initialization();
        loopModule = new Loop();
        initModule.initialize(hardwareMap, gamepad1);
    }

    @Override
    public void loop() {
        loopModule.handleLoop(initModule);
    }
}