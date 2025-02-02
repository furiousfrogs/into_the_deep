package org.firstinspires.ftc.teamcode.codeMethodized;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class VarAll {
    //All hardware init positions

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


    //from smth else
    public static float TransferATime = 0.7F;
    public static float TransferBTime = 0.5F;
    public static float TransferCTime = 0.3F;
    public static float TransferDTime = 0.5F;
    public static float OuttakeTime = 0.3F;
    public static float OuttakeTime2 = 0.5F;

    public static float inWristIntaking = 0.5F; //LIKELY TO CHANGE
    public static float inWristTransfer = 0.25F;

    public static float gateOpen = 0.1F;
    public static float gateClose = 0.35F;

    //Bigger number - intake is lower
    public static float inDown = 0.58F; //LIKELY TO CHANGE
    public static float inTransfer = 0.05F;
    public static float inIdle = 0.1F;

    //Bigger number - arm is lower
    public static float armOut = 0.85F;
    public static float armTransfer = 0.2F;
    public static float armSpec = 0.0F;
    public static float armSpecScore = 0.44F;

    //Bigger number - claw is open
    public static float clawClose = 0.63F;
    public static float clawOpen = 0.45F;
    public static float clawOpenWide = 0.3F;

    //Bigger number - wrist goes out
    public static float wristTransfer = 0.2F;
    public static float wristOut = 0.9F;
    public static float wristSpec = 0.2F;
    public static float wristSpecScore = 0.47F;
    public static float wristInit = 0F;

    //Slide Positions
    public static float slideDeposit = 2000F;
    public static float slideTransfer = 0F;
    public static float slideSpecPickup = 500F;
    public static float slideSpecScore = 1000F;


    // slide PID coefficients
    public static double kP = 0.005; // Proportional gain
    public static double kI = 0.0;  // Integral gain
    public static double kD = 0.00;  // Derivative gain
    public static double kF = 0.0; //Feedforward gain
    public static double maxIntegral = 800;
    public static double tolerance = 50;



    //pid state variables
    public static double targetPosition = 1000.0; // Desired slide position
    public static double lastError = 0; // Previous error for derivative calculation
    public static double integralSum = 0; // Accumulated integral

    //LED Values
    public static double LEDtest = 0.61; //RED

    //[frontLeft, frontRight, backLeft, backRight, leftIn, rightIn, outArm, wrist, claw, horSlide, vertSlideL, vertSlideR, intake, gate]
    static double[] initPos = {0.0,0.0,0.0,0.0,(double)inIdle,(double)inIdle,(double)armTransfer,(double)wristTransfer,(double)clawOpen,0.0,0.0,0.0,0.0,(double)gateClose};

}

