package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FFVar;

@TeleOp(name = "PIDtest", group= "TeleOp")
public class PIDtes extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo leftIn, rightIn, wrist, outArm, claw;
    private DcMotor horSlide, vertSlideL, vertSlideR, intake;

    double previousPower = 0;

    private boolean pidActive = false; // Tracks if PID is actively controlling
    private double targetPosition = 0.0; // Desired target position
    private double error, proportional, integral, derivative, power;
    private final double tolerance = 5.0; // Adjust based on your system (e.g., ±5 ticks)

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


    @Override
    public void init() {

        imu = hardwareMap.get(BHI260IMU.class, "imu");
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

    public void Position() {
        if (vertouch.isPressed()) {
            vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.triangle) {
            FFVar.targetPosition = 500;
        }
        if (gamepad1.circle) {
            FFVar.targetPosition = 800;
        }
    }

public void PID() {



    double integralSum = 0;

    double lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

    while (vertSlideR.getCurrentPosition() != FFVar.targetPosition) {


        // obtain the encoder position
        double encoderPosition = vertSlideR.getCurrentPosition();
        // calculate the error
        error = FFVar.targetPosition - encoderPosition;

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (FFVar.kP * error) + (FFVar.kI * integralSum) + (FFVar.kD * derivative);

        vertSlideR.setPower(out);
        vertSlideL.setPower(out);

        lastError = error;

        // reset the timer for next time
        timer.reset();

    }
}

    public void Telemetry() {
        telemetry.addData("Vertical Right Slide Pos", vertSlideR.getCurrentPosition());
        telemetry.addData("Vertical left Slide Pos", vertSlideL.getCurrentPosition());
        telemetry.addData("Horizontal Slide Pos", horSlide.getCurrentPosition());
        telemetry.addData("hor power", horSlide.getPower());
        telemetry.addData("ver left power", vertSlideL.getPower());
        telemetry.addData("ver right power", vertSlideR.getPower());

        if (intaking) {
            telemetry.addLine("intaking");
        }
        if (hortouch.isPressed()) {
            telemetry.addLine("touch joe");
        }
        if (vertouch.isPressed()) {
            telemetry.addLine("touch chuck");
        }
        if (leftIn.getPosition() == FFVar.InTransfer) {
            telemetry.addLine("Intake Up");
        } else if (leftIn.getPosition() == FFVar.InDown){
            telemetry.addLine("Intake down");
        }

        telemetry.addData("pos", vertSlideR.getCurrentPosition());
        telemetry.addData("target", FFVar.targetPosition);
        telemetry.update();
    }
    @Override
    public void loop () {
        Position();
        PID();
        Telemetry();

    }
}