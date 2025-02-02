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
    private Servo leftIn, rightIn, wrist, outArm, claw, gate, inWrist;
    private DcMotor horSlide, vertSlideL, vertSlideR, intake;
    double targetPosition = 0;


    boolean intakeAbort = false;
boolean botUnfold = false;

    private ElapsedTime abortTimer = new ElapsedTime();
    boolean intakeAction = false;
    private ElapsedTime intakeTimer = new ElapsedTime();

    boolean transferAction = false;
    private ElapsedTime transferTimer = new ElapsedTime();

    boolean outtakeAction = false;
    private ElapsedTime outtakeTimer = new ElapsedTime();

    boolean specScore = false;
    private ElapsedTime specScoreTimer = new ElapsedTime();

    boolean specAction = false;
    private ElapsedTime specTimer = new ElapsedTime();

    boolean specAction2 = false;
    private ElapsedTime specTimer2 = new ElapsedTime();

    boolean specAbort = false;
    private ElapsedTime specAbortTimer = new ElapsedTime();

private ElapsedTime initTimer = new ElapsedTime();
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

    public enum armState {
        armTransfering,
        armOuttaking,
        armSpec,
        armSpecScore,
        armIdle;
    }

    armState currentArmState = armState.armIdle;

    public enum intakeState {
        intakeIdle,
        intaking,
        intakeTransfering,
        intakeAbort;
    }

    intakeState currentIntakeState = intakeState.intakeIdle;

    public enum transferState {
        gateOpen,
        spin,
        idle;
    }

    transferState currentTransferState = transferState.gateOpen;
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

        inWrist = hardwareMap.get(Servo.class, "intakewrist");
        inWrist.setDirection(Servo.Direction.FORWARD);
        inWrist.setPosition(var.inWristIntaking);

        outArm = hardwareMap.get(Servo.class, "outarm");
        outArm.setDirection(Servo.Direction.FORWARD);
//        outArm.setPosition(var.armTransfer);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
//        wrist.setPosition(var.wristInit);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(var.gateClose);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
//        claw.setPosition(var.clawOpen);

        leftIn = hardwareMap.get(Servo.class, "leftin");
        leftIn.setDirection(Servo.Direction.FORWARD);
        leftIn.setPosition(var.inIdle);

        rightIn = hardwareMap.get(Servo.class, "rightin");
        rightIn.setDirection(Servo.Direction.FORWARD);
        rightIn.setPosition(var.inIdle);


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

    public void drive() {

        double turnvar = Math.max(1, 1 + (horSlide.getCurrentPosition() / 1000.0));

        boolean slow = gamepad1.options;
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx =  (gamepad1.right_trigger - gamepad1.left_trigger) / turnvar;


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
            frontLeftPower = frontLeftPower/2;
            frontRightPower = frontRightPower/2;
            backLeftPower = backLeftPower/2;
            backRightPower = backRightPower/2;
        }
        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }


    public void manualTake() {
        //manual controls
        if (!botUnfold) {
            initTimer.reset();
            claw.setPosition(var.clawClose);
            leftIn.setPosition(var.inDown);
            rightIn.setPosition(var.inDown);
            inWrist.setPosition(var.inWristIntaking);
            while (initTimer.seconds() < 0.3) {

            }
            outArm.setPosition(var.armTransfer);
            wrist.setPosition(var.wristTransfer);
            vertSlideL.setPower(1);
            vertSlideR.setPower(1);
            vertSlideL.setTargetPosition(250);
            vertSlideR.setTargetPosition(250);
            currentArmState = armState.armTransfering;
            while (initTimer.seconds() < 0.6) {

            }
            leftIn.setPosition(var.inTransfer);
            rightIn.setPosition(var.inTransfer);
            inWrist.setPosition(var.inWristTransfer);
            claw.setPosition(var.clawOpen);
            botUnfold = true;
        } else if (botUnfold) { //all the shit below

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            switch (currentArmState) {
                case armIdle:
                    if (currentGamepad1.circle && !previousGamepad1.circle) {
                        claw.setPosition(var.clawOpen);
                        outArm.setPosition(var.armTransfer);
                        wrist.setPosition(var.wristTransfer);
                        vertSlideL.setPower(1);
                        vertSlideR.setPower(1);
                        vertSlideL.setTargetPosition(250);
                        vertSlideR.setTargetPosition(250);
                        currentArmState = armState.armTransfering;

                    } else if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                        claw.setPosition(var.clawClose);
                        vertSlideL.setPower(1);
                        vertSlideR.setPower(1);
                        vertSlideL.setTargetPosition(1000);
                        vertSlideR.setTargetPosition(1000);
                        specAction = true;
                    }
                    break;
                case armTransfering:

                    if (currentGamepad1.circle && !previousGamepad1.circle) {
                        claw.setPosition(var.clawClose);
                        transferAction = true;
                        transferTimer.reset();

                    }

                    if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                        claw.setPosition(var.clawClose);
                        vertSlideL.setPower(1);
                        vertSlideR.setPower(1);
                        vertSlideL.setTargetPosition(1000);
                        vertSlideR.setTargetPosition(1000);
                        specAction = true;
                    }


                    break;
                case armOuttaking:

                    if (currentGamepad1.circle && !previousGamepad1.circle) {
                        claw.setPosition(var.clawOpen);
                        outtakeAction = true;
                        outtakeTimer.reset();

                    } else if (currentGamepad1.triangle && !previousGamepad1.triangle) { //unfinshed
                        claw.setPosition(var.clawClose);
                        vertSlideL.setPower(1);
                        vertSlideR.setPower(1);
                        vertSlideL.setTargetPosition(1000);
                        vertSlideR.setTargetPosition(1000);
                        specAction = true;
                    }
                    break;

                case armSpec:

                    if (!specAbort && currentGamepad1.triangle && !previousGamepad1.triangle) {
                        claw.setPosition(var.clawClose);
                        specAbort = true;
                        specAbortTimer.reset();
                    }
                    if (specAbort && specAbortTimer.seconds() < 0.5 && currentGamepad1.triangle && !previousGamepad1.triangle) {
                        claw.setPosition(var.clawOpenWide);
                        specAbort = false;
                    } else if (specAbort && specAbortTimer.seconds() > 0.5 && currentGamepad1.triangle && !previousGamepad1.triangle) {
                        specAbort = false;
                        claw.setPosition(var.clawClose);
                        specScore = true;
                        specScoreTimer.reset();
                    }
                    break;

                case armSpecScore:
                    if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                        claw.setPosition(var.clawOpen);
                        currentArmState = armState.armIdle;
                    }
                    break;
            }

            if (transferAction && transferTimer.seconds() > 0.5) {
                vertSlideL.setPower(1);
                vertSlideR.setPower(1);
                vertSlideL.setTargetPosition(2000);
                vertSlideR.setTargetPosition(2000);
                outArm.setPosition(var.armOut);
                wrist.setPosition(var.wristOut);
                currentArmState = armState.armOuttaking;
                transferAction = false;
            }

            if (outtakeAction && outtakeTimer.seconds() > 0.5) {

                outArm.setPosition(var.armTransfer);
                wrist.setPosition(var.wristTransfer);
                vertSlideL.setPower(1);
                vertSlideR.setPower(1);
                vertSlideL.setTargetPosition(250);
                vertSlideR.setTargetPosition(250);
                currentArmState = armState.armTransfering;
                outtakeAction = false;
            }
            if (specScore && specScoreTimer.seconds() > 0.5) {
                outArm.setPosition(var.armSpecScore);
                wrist.setPosition(var.wristSpecScore);
                currentArmState = armState.armSpecScore;
                specScore = false;
            }

            if (specAction && vertSlideL.getCurrentPosition() > 500) {
                outArm.setPosition(var.armSpec);
                wrist.setPosition(var.wristSpec);
                specAction2 = true;
                specAction = false;
                specTimer2.reset();
            }

            if (specAction2 && specTimer2.seconds() > 1) {
                claw.setPosition(var.clawOpenWide);
                currentArmState = armState.armSpec;
                specAction2 = false;
            }


            switch (currentIntakeState) {

                case intakeIdle:
                    intake.setPower(0); // Ensure motors are stopped in idle
                    if (currentGamepad1.cross && !previousGamepad1.cross) { // Start intake
                        gate.setPosition(var.gateClose);
                        leftIn.setPosition(var.inDown);
                        rightIn.setPosition(var.inDown);
                        inWrist.setPosition(var.inWristIntaking); // Set wrist to intaking position
                        currentIntakeState = intakeState.intaking; // Move to intaking state
                    }
                    break;

                case intaking:
                    intakeAbort = false;
                    // Control intake power with the left bumper
                    if (gamepad1.left_bumper) {
                        intake.setPower(-1); // Reverse intake
                    } else {
                        intake.setPower(1); // Normal intake
                    }

                    if (currentGamepad1.cross && !previousGamepad1.cross) { // Start transfer
                        leftIn.setPosition(var.inTransfer);
                        rightIn.setPosition(var.inTransfer);
                        inWrist.setPosition(var.inWristTransfer); // Set wrist to transfer position
                        abortTimer.reset(); // Reset abort timer
                        intakeAbort = true; // Set abort flag
                    }

                    //Abort logic
//                if (intakeAbort && abortTimer.seconds() < 0.7 && currentGamepad1.cross && !previousGamepad1.cross) {
//                    gate.setPosition(var.gateClose);
//                    leftIn.setPosition(var.inDown);
//                    rightIn.setPosition(var.inDown);
//                    inWrist.setPosition(var.inWristIntaking);
//                    intakeAbort = false;
//                    currentIntakeState = intakeState.intaking; // Reset to intaking state
//                    }

                    if (intakeAbort && abortTimer.seconds() > 0.5) {
                        resethor = true; // Trigger horizontal slide reset
                        inWrist.setPosition(var.inWristTransfer); // Ensure wrist is in transfer position
                        currentIntakeState = intakeState.intakeTransfering; // Transition to transfering state
                        intakeTimer.reset();
                        intakeAction = true;
                        intakeAbort = false; // Clear abort flag
                    }
                    break;

                case intakeTransfering:
                    transfering = true;

                    // Initialize transfer state if not already set
                    if (currentTransferState == null) {
                        currentTransferState = transferState.gateOpen;
                    }

                    // Handle transfer state logic
                    switch (currentTransferState) {
                        case gateOpen:
                            if (hortouch.isPressed()) { // Wait for touch sensor
                                gate.setPosition(var.gateOpen);
                                intakeTimer.reset();
                                currentTransferState = transferState.spin; // Move to spin state
                            }
                            break;

                        case spin:
                            intake.setPower(1); // Spin intake
                            if (intakeTimer.seconds() > 0.5) {
                                currentTransferState = transferState.idle; // Move to idle state
                            }
                            break;

                        case idle:
                            intake.setPower(0); // Stop motors
                            currentIntakeState = intakeState.intakeIdle; // Return to idle state
                            currentTransferState = transferState.gateOpen;
                            transfering = false; // Reset transfer flag
                            break;
                    }
                    break;
            }


// Reset vertical slides if touch sensor is pressed
            if (vertouch.isPressed() && vertSlideL.getCurrentPosition() != 0 && vertSlideR.getCurrentPosition() != 0) {
                vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

// Reset horizontal slide logic
        public void resethor() {
            if (!hortouch.isPressed() && resethor) {
                horSlide.setPower(-1); // Move slide backwards
            } else if (hortouch.isPressed() && resethor) {
                horSlide.setPower(0); // Stop slide
                resethor = false; // Reset the flag
            } else if (!resethor && !transfering) {
                horSlide.setPower(gamepad1.right_stick_x); // Allow manual control
            }
        }




    public void Telemetry() {
            telemetry.addData("vert", vertSlideL.getCurrentPosition());
            if (vertouch.isPressed()) {
                telemetry.addLine("touch joe");
            }
            telemetry.addData("arm state", currentArmState);
            telemetry.addData("intake state", currentIntakeState);
telemetry.addData("hor power", horSlide.getPower());
    }
    @Override
    public void loop () {
        drive();
        manualTake();
        Telemetry();
        resethor();
    }
}