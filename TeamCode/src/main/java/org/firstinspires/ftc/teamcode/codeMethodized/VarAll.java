package org.firstinspires.ftc.teamcode.codeMethodized;
import com.acmerobotics.dashboard.config.Config;
@Config
public class VarAll {
    //All hardware init positions

    //time
    public static float transferTime=0.0F; //maximum possible time it takes to move slides to transferposition
    public static float vertPosChangePosTime=0.0F; //time takes for vert slides to go from idle to pos change pos
    public static float resetTime=0.0F; //maximum time it takes for the whole arm to go to idle pos

    //arm
    public static float armIdle=0.0F;
    public static float armHuman=0.0F;
    public static float armScoring=0.0F;
    public static float armReset=0.0F;

    //wrist
    public static float wristIdle=0.0F;
    public static float wristHuman=0.0F;
    public static float wristScoring=0.0F;

    //claw
    public static float clawIdle=0.0F;
    public static float clawOpen=0.0F;
    public static float clawClose=0.0F;

    //intake box
    public static float intakeBoxUp=0.0F;
    public static float intakeBoxDown=0.0F;

    //vert slides
    public static float vertSlideIdle=0.0F;
    public static float vertSlidePosChange=0.0F;
    public static float vertSlideHuman=0.0F;
    public static float vertSlideScoring=0.0F;

    //hor slides
    public static float horSlideIdle=0.0F;
    public static float horSlideIn=0.0F;
    public static float horSlideTransfer=0.0F;

    //gate
    public static float gateOpen=0.0F;
    public static float gateClose=0.0F;

    //intake
    public static float intakeCW=-0.8F;
    public static float intakeCCW=0.8F;
    public static float intakeNoSpin=0.0F;


    //from smth else
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
    static double[] initPos = {0.0,0.0,0.0,0.0,(double)intakeBoxDown,(double)intakeBoxDown,(double)armIdle,(double)wristIdle,(double)clawIdle,(double)horSlideIdle,(double)vertSlideIdle,(double)vertSlideIdle,0.0,(double)gateClose};
}