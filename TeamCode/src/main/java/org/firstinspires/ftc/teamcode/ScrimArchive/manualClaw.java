package org.firstinspires.ftc.teamcode.ScrimArchive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FrogDriveUNO", group= "TeleOp")
public class manualClaw extends OpMode {
    private Servo arm, wrist, claw;
    private int armpos=0;
    private int wristpos=0;
    private int clawpos=0;
    @Override
    public void init() {
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(armpos);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(wristpos);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(clawpos);

    }

    @Override
    public void loop() {
        armpos += (int)gamepad1.right_stick_x;
        wristpos += (int)gamepad1.left_stick_x;
        double amount = gamepad1.left_trigger-gamepad1.right_trigger;
        clawpos += (int)amount;

        if (armpos<0){
            armpos=0;
        }
        else if (armpos>180){
            armpos=180;
        }
        if (wristpos<0){
            wristpos=0;
        }
        else if(wristpos>180){
            wristpos=180;
        }
        if (clawpos<0){
            clawpos=0;
        }
        else if (clawpos>180){
            clawpos=180;
        }


    }
}