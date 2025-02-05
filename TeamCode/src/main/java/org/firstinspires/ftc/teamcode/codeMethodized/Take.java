package org.firstinspires.ftc.teamcode.codeMethodized;


import com.qualcomm.robotcore.hardware.DcMotor;

public class Take {
    private float state = 0.0F;
    private float wait0 = 0.0F;
    private float wait1 = 0.0F;
    private float wait2 = 0.0F;
    private float wait3 = 0.0F;
    private float wait4 = 0.0F;

    public void manualTake(Initialization initModule) {
        if (initModule.horTouch.isPressed()) {
            initModule.horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            initModule.currentGod[9] = 0.0;
        }
        if (initModule.verTouch.isPressed()) {
            initModule.vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            initModule.vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            initModule.currentGod[10] = 0.0;
            initModule.currentGod[11] = 0.0;
        }

        if (initModule.currentGamepad2.cross && !initModule.previousGamepad2.cross) {
            initModule.currentGod[9] = VarAll.horSlideTransfer;
            wait0 = (float) (initModule.Timer.seconds() + VarAll.transferTime);
        }
        if (initModule.Timer.seconds() > wait0) {
            initModule.currentGod[6] = VarAll.clawClose;
            wait0 = Float.MAX_VALUE;
        }

        if (initModule.currentGamepad1.triangle && !initModule.previousGamepad1.triangle) {
            initModule.target = !initModule.target;
        }
        if (initModule.currentGamepad1.square && !initModule.previousGamepad1.square) {
            initModule.color = !initModule.color;
        }
        //manual controlled
        initModule.currentGod[9] -= initModule.currentGamepad2.left_stick_x;
        initModule.currentGod[10] -= initModule.currentGamepad2.left_stick_y;
        initModule.currentGod[11] -= initModule.currentGamepad2.left_stick_y;

        if (initModule.currentGamepad2.left_bumper && !initModule.previousGamepad2.left_bumper) {
            if (initModule.currentGod[12] > -0.2) {
                initModule.currentGod[12] = VarAll.intakeCW;
            } else {
                initModule.currentGod[12] = VarAll.intakeNoSpin;
            }
        }
        if (initModule.currentGamepad2.right_bumper && !initModule.previousGamepad2.right_bumper) {
            if (initModule.currentGod[12] < 0.2) {
                initModule.currentGod[12] = VarAll.intakeCCW;
            } else {
                initModule.currentGod[12] = VarAll.intakeNoSpin;
            }
        }
        if (initModule.currentGod[9] >= 750) {
            initModule.currentGod[4] = VarAll.intakeBoxDown;
            initModule.currentGod[5] = VarAll.intakeBoxDown;
        } else {
            initModule.currentGod[4] = VarAll.intakeBoxUp;
            initModule.currentGod[5] = VarAll.intakeBoxUp;
        }


        if (initModule.currentGamepad2.triangle && !initModule.previousGamepad2.triangle) {
            if (initModule.currentGod[8] == VarAll.clawOpen) {
                initModule.currentGod[8] = VarAll.clawClose;
            } else {
                initModule.currentGod[8] = VarAll.clawOpen;
            }
        }
        if (initModule.currentGamepad2.square && !initModule.previousGamepad2.square) {
            if (initModule.target) { //sample
                initModule.currentGod[10] = VarAll.vertSlideScoring;
                initModule.currentGod[11] = VarAll.vertSlideScoring;
                wait1 = (float) (initModule.Timer.seconds() + VarAll.vertPosChangePosTime);

            } else {
                initModule.currentGod[10] = VarAll.vertSlidePosChange;
                initModule.currentGod[11] = VarAll.vertSlidePosChange;
                wait1 = (float) (initModule.Timer.seconds() + VarAll.vertPosChangePosTime);
            }
        }
        if (initModule.Timer.seconds() > wait1) {
            if (initModule.target) {
                initModule.currentGod[6] = VarAll.armScoring;
                initModule.currentGod[7] = VarAll.wristScoring;
                wait1 = Float.MAX_VALUE;
            } else {
                initModule.currentGod[6] = VarAll.armHuman;
                initModule.currentGod[7] = VarAll.wristHuman;
                wait1 = Float.MAX_VALUE;
            }
        }
        if (initModule.currentGamepad2.circle && !initModule.previousGamepad2.circle) {
            if (initModule.currentGod[8] != VarAll.clawOpen) {
                initModule.currentGod[10] = VarAll.vertSlidePosChange;
                initModule.currentGod[11] = VarAll.vertSlidePosChange;
                wait2 = (float) (initModule.Timer.seconds() + VarAll.vertPosChangePosTime);
            } else {
                initModule.currentGod[10] = VarAll.vertSlideScoring;
                initModule.currentGod[11] = VarAll.vertSlideScoring;
            }
        }
        if (initModule.Timer.seconds() > wait2) {
            initModule.currentGod[6] = VarAll.armHuman;
            initModule.currentGod[7] = VarAll.wristHuman;
            initModule.currentGod[8] = VarAll.clawOpen;
            wait2 = Float.MAX_VALUE;
        }
        if (initModule.currentGamepad2.start && !initModule.previousGamepad2.start) {
            initModule.currentGod[10] = VarAll.vertSlidePosChange;
            initModule.currentGod[11] = VarAll.vertSlidePosChange;
            wait3 = (float) (initModule.Timer.seconds() + VarAll.vertPosChangePosTime);
        }
        if (initModule.Timer.seconds() > wait3) {
            initModule.currentGod[6] = VarAll.armIdle;
            initModule.currentGod[7] = VarAll.wristIdle;
            initModule.currentGod[8] = VarAll.clawIdle;
            wait3 = Float.MAX_VALUE;
            wait4 = (float) (initModule.Timer.seconds() + VarAll.resetTime);
        }
        if (initModule.Timer.seconds() > wait4) {
            initModule.currentGod[10] = VarAll.vertSlideIdle;
            initModule.currentGod[11] = VarAll.vertSlideIdle;
            wait4 = Float.MAX_VALUE;
        }
    }
}

