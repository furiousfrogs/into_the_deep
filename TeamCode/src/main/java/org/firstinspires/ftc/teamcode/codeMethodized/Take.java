package org.firstinspires.ftc.teamcode.codeMethodized;

private float state=0.0F;
private boolean direction=true; //true is positive (samples), false is negative (specimen)

public class Take {
    public void manualTake(Initialization initModule){
        if(initModule.currentGamepad1.triangle.ispressed()){
            initModule.target= !initModule.target;
        }
        if(initModule.currentGamepad1.square.isPressed()){
            initModule.color= !initModule.color;
        }
        //manual controlled
        if(initModule.currentGamepad2.left_bumper && !initModule.previousGamepad2.left_bumper){
            if(initModule.currentGod[13]>-0.2){
            initModule.currentGod[13]=-0.8;
            }else{
                initModule.currentGod[13]=0;
            }
        }
        if(initModule.currentGamepad2.right_bumper && !initModule.previousGamepad2.right_bumper){
            if(initModule.currentGod[13]<0.2){
                initModule.currentGod[13]=0.8;
                }else{
                    initModule.currentGod[13]=0;
                }
        }
        initModule.currentGod[9]=-initModule.currentGamepad2.left_stick_x;
        initModule.currentGod[10]=-initModule.currentGamepad2.left_stick_y;
        initModule.currentGod[11]=-initModule.currentGamepad2.left_stick_y;
        if(initModule.currentGamepad2.circle && ! initModule.previousGamepad2.circle){
            if(initModule.currentGod[12]=VarAll.inIdle){
                initModule.currentGod[12]=VarAll.inDown;
            }else{
                initModule.currentGod[12]=VarAll.inIdle;
            }
        }
        if(initModule.currentGamepad2.cross && !initModule.previousGamepad2.cross){
            initModule.currentGod[8]=VarAll.clawOpen;
        }
        
        //state controlled
        if(state=0.0F){

        }
        //safety controlled
        //reset controlled
    }
    
}
