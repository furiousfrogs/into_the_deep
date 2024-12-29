package org.firstinspires.ftc.teamcode.codeMethodized;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FFVar;

public class Initialization {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo leftIn, rightIn, wrist, outArm, claw;
    private DcMotor horSlide, vertSlideL, vertSlideR, intake;


    boolean sample = false;
    private ElapsedTime timer;
    // Declare globally to maintain state across iterations
    private ElapsedTime InitTime = new ElapsedTime();
    boolean InitAct = false;
    boolean Transfer1action = false;
    private ElapsedTime Transfer1Timer = new ElapsedTime();
    boolean Transfer2action = false;
    private ElapsedTime Transfer2Timer = new ElapsedTime();
    private ElapsedTime Transfer3Timer = new ElapsedTime();
    boolean Transfer3action = false;
    boolean OuttakeAction = false;
    private ElapsedTime OuttakeTimer = new ElapsedTime();
    private ElapsedTime Transfer4Timer = new ElapsedTime();
    boolean Transfer4action = false;

    boolean OuttakeAction2 = false;
    private ElapsedTime OuttakeTimer2 = new ElapsedTime();

    private int dynamicLimit = 1300;
    private boolean limitCalculated = false;

    boolean transfering = false;
    boolean outtaking = false;

    private BHI260IMU imu;

    private ColorRangeSensor coloursensor;
    private TouchSensor hortouch;
    private TouchSensor vertouch;
    boolean resethor = false;
    boolean resetver = false;
    boolean intaking = false;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    public void initialize(HardwareMap hardwareMap, Gamepad gamepad1) {
        imu=hardwareMap.get(BHI260IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)
        );


        imu.initialize(parameters);


        timer = new ElapsedTime();

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIn = hardwareMap.get(Servo.class, "leftin");
        leftIn.setDirection(Servo.Direction.FORWARD);


        rightIn = hardwareMap.get(Servo.class, "rightin");
        rightIn.setDirection(Servo.Direction.FORWARD);


        outArm = hardwareMap.get(Servo.class, "outarm");
        outArm.setDirection(Servo.Direction.FORWARD);
        outArm.setPosition(FFVar.ArmInit);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(FFVar.WristOut);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(FFVar.ClawClose);

        horSlide = hardwareMap.get(DcMotor.class, "righthor");
        horSlide.setDirection(DcMotor.Direction.REVERSE);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideL = hardwareMap.get(DcMotor.class, "leftvertical");
        vertSlideL.setDirection(DcMotor.Direction.FORWARD);
        vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideR = hardwareMap.get(DcMotor.class, "rightvertical");
        vertSlideR.setDirection(DcMotor.Direction.REVERSE);
        vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        hortouch = hardwareMap.get(TouchSensor.class, "hortouch");
        vertouch = hardwareMap.get(TouchSensor.class, "vertouch");
        coloursensor = hardwareMap.get(ColorRangeSensor.class, "coloursensor");
        timer.reset();



        while (timer.seconds() < 0.3) {
            // Wait for 1 second
        }
        // Move the intake up
        leftIn.setPosition(FFVar.InUp);
        rightIn.setPosition(FFVar.InUp);
    }
}
