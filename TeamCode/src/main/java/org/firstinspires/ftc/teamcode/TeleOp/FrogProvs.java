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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.FFVar;
import org.firstinspires.ftc.teamcode.var;

@TeleOp(name = "FrogProvs", group= "TeleOp")
public class FrogProvs extends OpMode {
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


        outArm = hardwareMap.get(Servo.class, "outarm");
        outArm.setDirection(Servo.Direction.FORWARD);
        outArm.setPosition(var.armTransfer);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(var.wristTransfer);



        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(var.ClawOpen);

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

    }
//        public void drive () {
//
//            double turnvar = Math.max(1, 1 + (horSlide.getCurrentPosition() / 1000.0));
//
//            boolean slow = gamepad1.options;
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = (gamepad1.right_trigger - gamepad1.left_trigger) / turnvar;
//
//
//            double slowvar = 2.0; // Slow mode divisor
//            double speedFactor = slow ? 1 / slowvar : 1;
//
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.8);
//
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//
//            double frontLeftPower = (y + x + rx) / denominator * speedFactor * 0.8;
//            double backLeftPower = (y - x + rx) / denominator * speedFactor * 0.8;
//            double frontRightPower = (y - x - rx) / denominator * speedFactor * 0.8;
//            double backRightPower = (y + x - rx) / denominator * speedFactor * 0.8;
//
//
//            // Normalize motor powers
//            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
//                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
//            if (maxPower > 1.0) {
//                frontLeftPower /= maxPower;
//                frontRightPower /= maxPower;
//                backLeftPower /= maxPower;
//                backRightPower /= maxPower;
//            }
//            if (vertSlideL.getCurrentPosition() > 2000) {
//                frontLeftPower = frontLeftPower / 2;
//                frontRightPower = frontRightPower / 2;
//                backLeftPower = backLeftPower / 2;
//                backRightPower = backRightPower / 2;
//            }
//            // Set motor powers
//            frontLeft.setPower(frontLeftPower);
//            frontRight.setPower(frontRightPower);
//            backLeft.setPower(backLeftPower);
//            backRight.setPower(backRightPower);
//        }
        public void manualTake () {
            //manual controls

            // Store previous state
            previousGamepad1.copy(currentGamepad1);
            // Update current state with the latest gamepad data
            currentGamepad1.copy(gamepad1);

            // Store previous state
            previousGamepad2.copy(currentGamepad2);
            // Update current state with the latest gamepad data
            currentGamepad2.copy(gamepad2);
            if (vertSlideL.getCurrentPosition() > 1000) {
                outArm.setPosition(var.armOut);
                wrist.setPosition(var.wristOut);
            } else if (vertSlideL.getCurrentPosition() < 1000 && vertSlideL.getPower() < 0 ) {
                outArm.setPosition(var.armTransfer);
                wrist.setPosition(var.wristTransfer);

            }
if (vertouch.isPressed()) {
    vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
            if ((currentGamepad1.cross && !previousGamepad1.cross)  && claw.getPosition() == var.ClawOpen) {
                claw.setPosition(var.ClawClose);
            } else if (currentGamepad1.cross && !previousGamepad1.cross) {
                claw.setPosition(var.ClawOpen);
            }
            vertSlideL.setPower(gamepad1.right_stick_x);
            vertSlideR.setPower(gamepad1.right_stick_x);

            horSlide.setPower(gamepad1.left_stick_x);

// Clamp target position to safe limits

        }

    public void Telemetry() {
            telemetry.addData("vert", vertSlideL.getCurrentPosition());


    }
    @Override
    public void loop () {
//        drive();
        manualTake();
        Telemetry();
    }
}