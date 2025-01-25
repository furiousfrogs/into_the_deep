package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
@Config
public class var {
    //Time
    public static float TransferATime = 0.7F;
    public static float TransferBTime = 0.5F;
    public static float TransferCTime = 0.3F;
    public static float TransferDTime = 0.5F;
    public static float OuttakeTime = 0.3F;
    public static float OuttakeTime2 = 0.5F;

    public static float gateOpen = 0F;
    public static float gateClose = 0.3F;

    //Bigger number - intake is lower
    public static float inDown = 0.69F;
    public static float intransfer = 0.1F;
    public static float inIdle = 0.4F;

    //Bigger number - arm is lower
    public static float armOut = 0.85F;
    public static float armTransfer = 0.4F;
    public static float armSpec = 0.02F;
    public static float armSpecScore = 0.5F;

    //Bigger number - claw is open
    public static float clawClose = 0.63F;
    public static float clawOpen = 0.45F;
    public static float clawOpenWide = 0.3F;

    //Bigger number - wrist goes out
    public static float wristTransfer = 0.23F;
    public static float wristOut = 1F;
    public static float wristSpec = 0.3F;
    public static float wristSpecScore = 0.55F;

    //Slide Positions
    public static float slideDeposit = 1300F;
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
}
