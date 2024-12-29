package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "lightstest", group= "TeleOp")
public class lightstest extends OpMode {
    private RevBlinkinLedDriver lights;
    @Override
    public void init() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }
    @Override
    public void loop() {
lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
}
