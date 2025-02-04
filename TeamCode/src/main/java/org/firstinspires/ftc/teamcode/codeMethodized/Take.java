package org.firstinspires.ftc.teamcode.codeMethodized;


public class Take {
    private float state=0.0F;
    public void manualTake(Initialization initModule){
        if(initModule.horTouch.isPressed()){
            initModule.horSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            initModule.horSlide.setPower(0);
            initModule.horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            initModule.currentGod[9]=0.0;
        }
        if(initModule.verTouch.isPressed())}{
            initModule.horSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            initModule.verTouch.setPower(0);
            initModule.verTouch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.verTouch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            initModule.currentGod[10]=0.0;
            initModule.currentGod[11]=0.0;
        }
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
        initModule.currentGod[9]-=initModule.currentGamepad2.left_stick_x;
        initModule.currentGod[10]-=initModule.currentGamepad2.left_stick_y;
        initModule.currentGod[11]-=initModule.currentGamepad2.left_stick_y;
        if(initModule.currentGamepad2.triangle && !initModule.previousGamepad2.triangle){
            if (initModule.currentGod[8]==VarAll.clawOpen){
                initModule.currentGod[8]=VarAll.clawClose;
            }else{
                initModule.currentGod[8]=VarAll.clawOpen
            }
        }
        if(initModule.currentGamepad2.square && !initModule.previousGamepad2.square){

        }
    }

