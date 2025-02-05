package org.firstinspires.ftc.teamcode.codeMethodized;
import org.firstinspires.ftc.teamcode.codeMethodized.VarAll;
public class Communicate {
    public void Talk(Initialization initModule){
        initModule.frontLeft.setPower(initModule.currentGod[0]);

        initModule.frontRight.setPower(initModule.currentGod[1]);

        initModule.backLeft.setPower(initModule.currentGod[2]);

        initModule.backRight.setPower(initModule.currentGod[3]);
        
        if(initModule.currentGod[4]>=0.0){
            initModule.leftIn.setPosition(initModule.currentGod[4]);
        }

        if(initModule.currentGod[5]>=0.0){
            initModule.rightIn.setPosition(initModule.currentGod[5]);
        }

        if(initModule.currentGod[6]>=0.0){
            initModule.outArm.setPosition(initModule.currentGod[6]);
        }

        if(initModule.currentGod[7]>=0.0){
            initModule.wrist.setPosition(initModule.currentGod[7]);
        }

        if(initModule.currentGod[8]>=0.0){
            initModule.claw.setPosition(initModule.currentGod[8]);
        }

        if(initModule.currentGod[9]>=0.0){
            initModule.horSlide.setTargetPosition((int)initModule.currentGod[9]);
        }

        if(initModule.currentGod[10]>=0.0){
            initModule.vertSlideL.setTargetPosition((int)initModule.currentGod[10]);
        }

        if(initModule.currentGod[11]>=0.0){
            initModule.vertSlideR.setTargetPosition((int)initModule.currentGod[11]);
        }
        
        initModule.intake.setPower((int) initModule.currentGod[12]);

        initModule.gate.setPosition(initModule.currentGod[13]);


    }
    public void Listen(Initialization initModule){

    }
}

