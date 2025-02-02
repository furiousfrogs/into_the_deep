package org.firstinspires.ftc.teamcode.codeMethodized;
import org.firstinspires.ftc.teamcode.codeMethodized.VarAll;
public class Communicate {
    public void Talk(Initialization initModule){
        initModule.frontLeft.setPower(initModule.currentGod[0]);

        initModule.frontRight.setPower(initModule.currentGod[1]);

        initModule.backLeft.setPower(initModule.currentGod[2]);

        initModule.backRight.setPower(initModule.currentGod[3]);

        initModule.leftIn.setPosition(initModule.currentGod[4]);

        initModule.rightIn.setPosition(initModule.currentGod[5]);

        initModule.outArm.setPosition(initModule.currentGod[6]);

        initModule.wrist.setPosition(initModule.currentGod[7]);

        initModule.claw.setPosition(initModule.currentGod[8]);

        initModule.horSlide.setTargetPosition((int)initModule.currentGod[9]);

        initModule.vertSlideL.setTargetPosition((int)initModule.currentGod[10]);

        initModule.vertSlideR.setTargetPosition((int) initModule.currentGod[11]);

        initModule.intake.setTargetPosition((int) initModule.currentGod[12]);

        initModule.gate.setPosition(initModule.currentGod[13]);


    }
    public void Listen(Initialization initModule){

    }
}

