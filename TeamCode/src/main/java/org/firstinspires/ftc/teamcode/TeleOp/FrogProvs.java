package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
double targetPosition = 0;

    double transferTime = 0.2;
boolean abort = false;
    boolean specReady = false;
    boolean spec = false;
    boolean sample = true;
    boolean slidesUp = false;
    boolean slidesDown = true;
    boolean slidesSpecDown = false;
boolean slidesSpecUp = false;


    boolean transferAction = false;
    private ElapsedTime transferTimer = new ElapsedTime();

    boolean outtakeAction = false;
    private ElapsedTime outtakeTimer = new ElapsedTime();

    boolean specAction = false;
    private ElapsedTime specTimer = new ElapsedTime();

    boolean specAction2 = false;
    private ElapsedTime specTimer2 = new ElapsedTime();


    private int dynamicLimit = 1300;
    private boolean limitCalculated = false;

    boolean transfering = false;
    boolean outtaking = false;

    private BHI260IMU imu;

//    private ColorRangeSensor coloursensor;
    private TouchSensor hortouch;
    private TouchSensor vertouch;
    boolean resethor = false;
    boolean resetver = false;
    boolean intaking = false;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;


boolean pidActive = false;

    @Override
    public void init() {


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
        claw.setPosition(var.clawOpen);

        horSlide = hardwareMap.get(DcMotor.class, "righthor");
        horSlide.setDirection(DcMotor.Direction.REVERSE);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideL = hardwareMap.get(DcMotor.class, "leftvertical");
        vertSlideL.setDirection(DcMotor.Direction.FORWARD);
        vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideL.setTargetPosition(0);
        vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideR = hardwareMap.get(DcMotor.class, "rightvertical");
        vertSlideR.setDirection(DcMotor.Direction.REVERSE);
        vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideR.setTargetPosition(0);
        vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
//        coloursensor = hardwareMap.get(ColorRangeSensor.class, "coloursensor");

    }
        public void drive () {

            double turnvar = Math.max(1, 1 + (horSlide.getCurrentPosition() / 1000.0));

            boolean slow = gamepad1.options;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = (gamepad1.right_trigger - gamepad1.left_trigger) / turnvar;


            double slowvar = 2.0; // Slow mode divisor
            double speedFactor = slow ? 1 / slowvar : 1;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.8);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double frontLeftPower = (y + x + rx) / denominator * speedFactor * 0.8;
            double backLeftPower = (y - x + rx) / denominator * speedFactor * 0.8;
            double frontRightPower = (y - x - rx) / denominator * speedFactor * 0.8;
            double backRightPower = (y + x - rx) / denominator * speedFactor * 0.8;


            // Normalize motor powers
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }
            if (vertSlideL.getCurrentPosition() > 2000) {
                frontLeftPower = frontLeftPower / 2;
                frontRightPower = frontRightPower / 2;
                backLeftPower = backLeftPower / 2;
                backRightPower = backRightPower / 2;
            }
            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }


        
        public void manualTake () {
            //manual controls

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

//circle is the outtake button
            // if sample boolean is true, one press will bring the arm up and another press will lower it

            if (currentGamepad1.circle && !previousGamepad1.circle && sample && !slidesUp) {   //this will raise the slides to the deposit position. the timer is caused so there is a 0.2 second delay between when you grab the sample and when u move the slides.
                claw.setPosition(var.clawClose);
                transferAction = true;
                transferTimer.reset();
            } else if (currentGamepad1.circle && !previousGamepad1.circle && !slidesDown && !slidesSpecDown) { //this is used to lower the slides to the transfer position. it only works if the slide is at the deopis position. THIS SHOULD BE THE DEFAULT POSITION THE CLAW CAN RETURN TO
                claw.setPosition(var.clawOpen);
                outtakeAction = true;
                outtakeTimer.reset();
                transferTime = 0.2;
            } else if (currentGamepad1.circle && !previousGamepad1.circle && slidesSpecDown) { //this is to return the arm to the deposit position when the arm is in the pickup sample position. it needs to be different because there has to be a delay between closing the claw and moving the arm.  THIS IS VERY IMPORTANT OR ELSE THE CLAW WILL BREAK
                outArm.setPosition(var.armTransfer);
                wrist.setPosition(var.wristTransfer);
                claw.setPosition(var.clawClose);
                outtakeAction = true;
                outtakeTimer.reset();
                transferTime = 0.4;
            } else if (currentGamepad1.circle && !previousGamepad1.circle && slidesSpecUp) { //this is for lowering the arm to the transfer position. it is the same as the default position i just dk how to do it
                outtakeAction = true;
                outtakeTimer.reset();
                spec = false;
                specReady = false;
            }

            if (transferAction && transferTimer.seconds() > 0.2) { //this timer is for the depositing. 0.2 seconds after the button for deposit is pressed, this will move. THIS IS IMPORTANT TO ENSURE THE CLAW PICKS UP A SAMPLE BEFORE MOVING
                vertSlideL.setPower(1);
                vertSlideR.setPower(1);
                vertSlideL.setTargetPosition(1800);
                vertSlideR.setTargetPosition(1800);
                outArm.setPosition(var.armOut);
                wrist.setPosition(var.wristOut);
                slidesDown = false;
                slidesUp = true;
                transferAction = false;
            }

            if (outtakeAction && outtakeTimer.seconds() > transferTime) { //this is for trabnsfering. the 0.2 second is necessary to ensure that the claw has time to drop the samples before moving. when this button is pressed when the claw is picking up a specimen THE 0.4 SECOND WAIT IS REQUIRED OR ELSE THE CLAW WILL BREAK
                outArm.setPosition(var.armTransfer);
                wrist.setPosition(var.wristTransfer);
                vertSlideL.setPower(1);
                vertSlideR.setPower(1);
                vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlideL.setTargetPosition(0);
                vertSlideR.setTargetPosition(0);
                slidesDown = true;
                slidesUp = false;
                outtakeAction = false;
                claw.setPosition(var.clawOpen);
               // sample = false;
            }


            //triangle is the specimen scoring position
            if (currentGamepad1.triangle && !previousGamepad1.triangle) { //all triangles are here
                if (!slidesSpecDown && !spec && !specReady) { //this brings the arm to the spec pickup
                    claw.setPosition(var.clawClose);
                    vertSlideL.setPower(1);
                    vertSlideR.setPower(1);
                    vertSlideL.setTargetPosition(1000);
                    vertSlideR.setTargetPosition(1000);
                    outArm.setPosition(var.armSpec);
                    wrist.setPosition(var.wristSpec);
                    specTimer.reset();
                    specAction = true;


                } else if (slidesSpecDown && !slidesSpecUp && !spec && !specReady) { //this brings the arm to the spec score

                    claw.setPosition(var.clawClose);
                    specTimer2.reset();
                    specAction2 = true;
                    spec = true;
                    abort = false;

                } else if (slidesSpecUp && specReady) { //this releases the spec to score it
                    claw.setPosition(var.clawOpen);
                    specReady = false;
                    spec = false;
                }
            }

            if (specAction && specTimer.seconds() > 0.5) { //this is for when the arm moves from default to spec pickup. THIS 0.5 SECOND WAIT BEFORE REOPENING THE CLAW IS REQUIRED OR ELSE THE CLAW WILL BREAK
                claw.setPosition(var.clawOpenWide);
                slidesSpecDown = true;
                slidesSpecUp = false;
                specAction = false;
            }
            if (specAction2 && specTimer2.seconds() > 1 && !abort) { //this is the code to bring the arm to the scoring position. THE 1 SECOND WAIT IS REQUIRED BEFORE MOVING THE ARM OR ELSE THE CLAW WILL BREAK
                outArm.setPosition(var.armSpecScore);
                wrist.setPosition(var.wristSpecScore);
                vertSlideL.setPower(1);
                vertSlideR.setPower(1);
                vertSlideL.setTargetPosition(1600);
                vertSlideR.setTargetPosition(1600);
                slidesSpecUp = true;
                slidesSpecDown = false;
                specReady = true;
                specAction2 = false;
            } else if (currentGamepad1.triangle && !previousGamepad1.triangle && !abort && specTimer2.seconds() < 1 && specTimer2.seconds() > 0.1 && specAction2) { //this is an abort. when the claw is at the specimen pickup position and you press triangle again. If u miss, you have 1 second to abort. it will reset you to the specimen pickup position
                abort = true;
                slidesSpecDown = true;
                slidesSpecUp = false;
                specReady = false;
                spec = false;
                specAction2 = false;
                claw.setPosition(var.clawOpenWide);

            }

            if (vertouch.isPressed() && vertSlideL.getCurrentPosition() != 0 && vertSlideR.getCurrentPosition() != 0) { //slide reset
                vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            horSlide.setPower(gamepad1.left_stick_x);
            }




    public void Telemetry() {
            telemetry.addData("vert", vertSlideL.getCurrentPosition());
            if (vertouch.isPressed()) {
                telemetry.addLine("touch joe");
            }

    }
    @Override
    public void loop () {
        drive();
        manualTake();
        Telemetry();
    }
}