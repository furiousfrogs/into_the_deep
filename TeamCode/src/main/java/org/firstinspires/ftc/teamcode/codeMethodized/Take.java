package org.firstinspires.ftc.teamcode.codeMethodized;


public class Take {
    private float state=0.0F;
    private boolean direction=true;//true is positive (samples), false is negative (specimen)
    public void manualTake(Initialization initModule){
        if(initModule.currentGamepad1.triangle){
            initModule.target= !initModule.target;
        }
        if(initModule.currentGamepad1.square){
            initModule.color= !initModule.color;
        }
        //manual controlled
        if(initModule.currentGamepad2.left_bumper && !initModule.previousGamepad2.left_bumper){
            if(initModule.currentGod[12]>-0.2){
            initModule.currentGod[12]=-0.8;
            }else{
                initModule.currentGod[12]=0;
            }
        }
        if(initModule.currentGamepad2.right_bumper && !initModule.previousGamepad2.right_bumper){
            if(initModule.currentGod[12]<0.2){
                initModule.currentGod[12]=0.8;
                }else{
                    initModule.currentGod[12]=0;
                }
        }
        initModule.currentGod[9]=-initModule.currentGamepad2.left_stick_x;
        initModule.currentGod[10]=-initModule.currentGamepad2.left_stick_y;
        initModule.currentGod[11]=-initModule.currentGamepad2.left_stick_y;
        if(initModule.currentGamepad2.triangle && !initModule.previousGamepad2.triangle){
            initModule.currentGod[8]=VarAll.clawOpen;
        }
        if(initModule.currentGamepad2.square && !initModule.previousGamepad2.square){

        }
    }
    
}

