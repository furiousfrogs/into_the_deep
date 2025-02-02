package org.firstinspires.ftc.teamcode.codeMethodized;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class VarAll {
    //All hardware init positions
//[frontLeft, frontRight, backLeft, backRight, leftIn, rightIn, outArm, wrist, claw, horSlide, vertSlideL, vertSlideR, intake, gate]
    static double[] initPos = {0.0,0.0,0.0,0.0,(double)inIdle,(double)inIdle,(double)armTransfer,(double)wristTransfer,(double)clawOpen,0.0,0.0,0.0,0.0,(double)gateClose};

    //time
    public static float in2SampleTime=0.0F;
    public static float in2HumanTime=0.0F;
    public static float human2SpecimenTime=0.0F;

    //arm
    public static float armIdle=0.0F;
    public static float armGrab=0.0F;
    public static float armSample=0.0F;
    public static float armSpecimen=0.0F;
    public static float armHuman=0.0F;

    //wrist
    public static float wristIdle=0.0F;
    public static float wristGrab=0.0F;
    public static float wristSample=0.0F;
    public static float wristSpecimen=0.0F;
    public static float wristHuman=0.0F;
    //claw
    public static float clawIdle=0.0F;
    public static float clawGrab=0.0F;
    public static float clawSample=0.0F;
    public static float clawSpecimen=0.0F;
    public static float clawHuman=0.0F;

    //intake box
    public static float inIdle=0.0F;
    public static float inDown=0.0F;
    public static float inTransfer=0.0F;

    //vert slides
    public static float vertSlideIdle=0.0F;
    public static float vertSlideSample=0.0F;
    public static float vertSlideHuman=0.0F;
    public static float vertSlideSpecimen=0.0F;

    //hor slides
    public static float horSlideIdle=0.0F;
    public static float horSlideIn=0.0F;
    public static float horSlideTransfer=0.0F;

    //gate
    public static float gateOpen = 0F;
    public static float gateClose = 0.3F;
}

