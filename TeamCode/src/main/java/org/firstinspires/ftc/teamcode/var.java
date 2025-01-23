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
    //Bigger number - intake is lower
    public static float InDown = 0.565F;
    public static float InUp = 0.2F;
    public static float InTransfer = 0.235F;
    public static float InWait = 0.44F; //this is the position where the intake arm waits for the outtake arm to go down
    public static float InWait2 = 0.3F; // this is for square when slide is retracted

    //Bigger number - arm is lower
    public static float armOut = 0.85F;
    public static float armTransfer = 0.4F;
    public static float armSpec = 0.02F;


    //Bigger number - claw is open
    public static float ClawClose = 0.63F;
    public static float ClawOpen = 0.45F;

    //Bigger number - wrist goes out
    public static float wristTransfer = 0.23F;
    public static float wristOut = 1F;
    public static float wristSpec = 0.3F;

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
