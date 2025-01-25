package org.firstinspires.ftc.teamcode.codeMethodized;

public class Drive {
    public void Drive(Initialization initModule){
        double y=-initModule.currentGamepad1.left_stick_y;
        double x=initModule.currentGamepad1.left_stick_x*1.3; //TUNE THE COUNTER STRAFING
        double rx= (initModule.currentGamepad1.right_trigger-initModule.currentGamepad1.left_trigger);
        double denominator=Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
        double frontLeftPower = (y+x+rx)/denominator;
        double backLeftPower = (y-x+rx)/denominator;
        double frontRightPower = (t-x-rx)/denominator;
        double backRightPower = (y+x-rx)/denominator;
        initModule.currentGod[0]=frontLeftPower;
        initModule.currentGod[1]=backLeftPower;
        initModule.currentGod[2]=frontRightPower;
        initModule.currentGod[3]=backRightPower;
    }
    
}
