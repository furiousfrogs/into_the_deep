package org.firstinspires.ftc.teamcode.codeMethodized;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Initialization {

    public DcMotor motor;
    public Servo servoL,servoR;
    public Gamepad gamepad;

    public void initialize(HardwareMap hardwareMap, Gamepad gamepad1) {
        motor = hardwareMap.get(DcMotor.class, "righthor");
        servoL = hardwareMap.get(Servo.class, "leftin");
        servoR=hardwareMap.get(Servo.class,"rightin");
        gamepad = gamepad1;
    }
}
