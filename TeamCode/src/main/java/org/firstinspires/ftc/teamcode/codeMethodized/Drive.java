package org.firstinspires.ftc.teamcode.codeMethodized;

public class Drive {
    public void Drive(Initialization initModule){

        //Taking input
        double y=-initModule.currentGamepad1.left_stick_y;
        double x= initModule.currentGamepad1.left_stick_x*1.3; 

        //Adjusting scale
        if (x > -0.2 && x < 0.2) {
            x = 0;
        }
        if (y > -0.2 && y < 0.2) {
            y = 0;
        }
        
        
        double rx= (initModule.currentGamepad1.right_trigger-initModule.currentGamepad1.left_trigger);


        double denominator=Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
        
        //set constant to max power if its above a certain x limit
        double constant = 0;
        if (Math.abs(x) >  0.8){
            constant = 1.0;
        }

        double frontLeftPower = Math.max((y+x+rx)/denominator,constant);
        double backLeftPower = Math.max((y-x+rx)/denominator,constant);
        double frontRightPower = Math.max((y-x-rx)/denominator,constant);
        double backRightPower = Math.max((y+x-rx)/denominator,constant);



        initModule.currentGod[0]=frontLeftPower;
        initModule.currentGod[1]=backLeftPower;
        initModule.currentGod[2]=frontRightPower;
        initModule.currentGod[3]=backRightPower;
    }
}
