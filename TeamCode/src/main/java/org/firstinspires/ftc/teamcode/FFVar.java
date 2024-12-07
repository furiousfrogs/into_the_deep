package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
@Config
public class FFVar {
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
    public static float InTransfer = 0.25F;
    public static float InWait = 0.44F; //this is the position where the intake arm waits for the outtake arm to go down
    public static float InWait2 = 0.3F; // this is for square when slide is retracted

    //Bigger number - arm is lower
    public static float ArmOut2 = 0.09F;
    public static float ArmOut = 0.17F;
    public static float ArmTransfer = 0.39F;
    public static float ArmInit = 0.3F;
    public static float ArmWait = 0.3F;

    //Bigger number - claw is open
    public static float ClawClose = 0.6F;
    public static float ClawOpen = 0.45F;

    //Bigger number - wrist goes out
    public static float WristTransfer = 0.12F;
    public static float WristOut = 0.7F;
    public static float WristWait = 0.4F;
    public static float WristTransfer2 = 0.2F;

    // PID coefficients
    public static float kP = 0.01F; // Proportional gain
    public static float kI = 0.0F;  // Integral gain
    public static float kD = 0.0F;  // Derivative gain

    //pid state variables
    public static float targetPosition = 0F; // Desired slide position
    public static float lastError = 0F; // Previous error for derivative calculation
    public static float integralSum = 0F; // Accumulated integral
}
