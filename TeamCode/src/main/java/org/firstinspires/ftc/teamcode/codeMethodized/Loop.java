package org.firstinspires.ftc.teamcode.codeMethodized;

public class Loop {

    public void handleLoop(Initialization initModule) {
        if (initModule.gamepad.triangle) {
            initModule.motor.setPower(1.0);
        } else {
            initModule.motor.setPower(0.0);
        }

        if (initModule.gamepad.square) {
            initModule.servoL.setPosition(0.2F);
            initModule.servoR.setPosition(0.2F);
        } else {
            initModule.servoL.setPosition(0.565F);
            initModule.servoR.setPosition(0.565F);
        }
    }
}
