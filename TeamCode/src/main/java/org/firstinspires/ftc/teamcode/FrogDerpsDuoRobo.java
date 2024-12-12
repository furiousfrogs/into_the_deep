package org.firstinspires.ftc.teamcode;
import android.drm.DrmStore;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.InternalWebSocketCommandException;

@TeleOp(name = "FrogDerpsDuoRobo", group= "TeleOp")
public class FrogDerpsDuoRobo extends OpMode {
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
        int red = coloursensor.red();
        int green = coloursensor.green();
        int blue = coloursensor.blue();
        // Store previous state
        previousGamepad1.copy(currentGamepad1);
        // Update current state with the latest gamepad data
        currentGamepad1.copy(gamepad1);

        // Store previous state
        previousGamepad2.copy(currentGamepad2);
        // Update current state with the latest gamepad data
        currentGamepad2.copy(gamepad2);

        if (gamepad2.options && !intaking && !outtaking) {
            outArm.setPosition(FFVar.ArmOut);
            wrist.setPosition(FFVar.WristOut);
            claw.setPosition(FFVar.ClawOpen);
        }

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) { //Intake
            if (intake.getPower() < 0.2) {
                intake.setPower(0.8);
                intaking = false;
            } else {
                intake.setPower(0);
                intaking = false;
            }
        }


        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (intake.getPower() > -0.2) {
                intake.setPower(-0.8); // Reverse
                intaking = true;
            } else {
                intake.setPower(0); // Stop
                intaking = false;
            }
        }

        if (intaking && red > green && red > blue && red > 500) { //detects red
            intake.setPower(0);
            intaking = false;
        }
        if (intaking && blue > red && blue > green && blue > 500) { //detects blue
            intake.setPower(0);
            intaking = false;
        }


        if (intaking && red > 120 && green > 120 && blue < 120) { //detects yellow
            intake.setPower(0);
            intaking = false;
        }


        if (currentGamepad2.cross && !previousGamepad2.cross && !transfering) { // Arm down
            if (leftIn.getPosition() > 0.5) {
                // Set servo positions to ArmDwn
                leftIn.setPosition(FFVar.InWait);
                rightIn.setPosition(FFVar.InWait);
            } else if ((leftIn.getPosition() < 0.5) && Math.abs(horSlide.getCurrentPosition()) > 350) {
                // Set servo positions to ArmUp
                leftIn.setPosition(FFVar.InDown);
                rightIn.setPosition(FFVar.InDown);
            }
        }


        if (hortouch.isPressed()) { //Horizontal touch sensor detection
            horSlide.setPower(0);
            horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (vertouch.isPressed()) { //Reset vertical encoders
            vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (horSlide.getCurrentPosition() < 550 && leftIn.getPosition() > FFVar.InWait) {
            leftIn.setPosition(FFVar.InWait);
            rightIn.setPosition(FFVar.InWait);
        }

        if (currentGamepad2.circle && !previousGamepad2.circle && sample) {
            outtaking = true;
            // Start the Outtake action
            outArm.setPosition(FFVar.ArmOut2);
            wrist.setPosition(FFVar.WristOut);
            claw.setPosition(FFVar.ClawClose);
            OuttakeTimer.reset(); // Reset the timer
            OuttakeAction = true; // Mark the action as in progress
        }

// Check if Outtake action is in progress
        if (OuttakeAction) {
            if (OuttakeTimer.seconds() >= FFVar.OuttakeTime) {
                // Perform the next step of the action
                claw.setPosition(FFVar.ClawOpen);
                OuttakeAction = false; // End the action
                OuttakeAction2 = true;
                OuttakeTimer2.reset();
            }
        }

        if (OuttakeAction2) {
            if (OuttakeTimer2.seconds() >= FFVar.OuttakeTime2) {
                claw.setPosition(FFVar.ClawClose);
                outArm.setPosition(FFVar.ArmWait);
                wrist.setPosition(FFVar.WristOut);

                OuttakeAction2 = false;
                sample = false;
                outtaking = false;
            }
        }

        if ((currentGamepad2.triangle && !previousGamepad2.triangle) || (currentGamepad1.square && !previousGamepad1.square)) {
            if (!hortouch.isPressed()) {
                horSlide.setPower(-1);
            }
            if (!vertouch.isPressed()) {
                vertSlideR.setPower(-1);
                vertSlideL.setPower(-1);
            }
            resethor = true;
            resetver = true;
        }

        if (((currentGamepad2.square && !previousGamepad2.square) || (currentGamepad1.square && !previousGamepad1.square))  && !transfering && !outtaking) {
            transfering = true;
            if (horSlide.getCurrentPosition() >= 350) {
                leftIn.setPosition(FFVar.InWait);
                rightIn.setPosition(FFVar.InWait);
            } else {
                leftIn.setPosition(FFVar.InWait2);
                rightIn.setPosition(FFVar.InWait2);
            }
            if (!hortouch.isPressed()) {
                horSlide.setPower(-1);
            }
            if (!vertouch.isPressed()) {
                vertSlideR.setPower(-1);
                vertSlideL.setPower(-1);
            }
            // Move the arm and wrist to their positions
            outArm.setPosition(FFVar.ArmTransfer);
            wrist.setPosition(FFVar.WristTransfer);
            claw.setPosition(FFVar.ClawOpen);
            intake.setPower(0);


            // Initialize reset flags
            resethor = true;
            resetver = true;


            Transfer1action = true; // Start a new action sequence
            Transfer1Timer.reset();
        }

// Check if an action is in progress
        if (Transfer1action) {
            if (hortouch.isPressed() && vertouch.isPressed() && Transfer1Timer.seconds() > FFVar.TransferATime) {

                leftIn.setPosition(FFVar.InTransfer);
                rightIn.setPosition(FFVar.InTransfer);

                // Set the positions for leftIn and rightIn after delay


                Transfer2action = true;
                Transfer2Timer.reset();

                // End the action
                Transfer1action = false;
            }
        }
        if (Transfer2action) {

            if (Transfer2Timer.seconds() >= FFVar.TransferBTime) {
                claw.setPosition(FFVar.ClawClose);


                Transfer3action = true;
                Transfer3Timer.reset();

                // End the action
                Transfer2action = false;
            }
        }
        if (Transfer3action) {
            if (Transfer3Timer.seconds() >= FFVar.TransferCTime) {

                outArm.setPosition(FFVar.ArmWait);
                wrist.setPosition(FFVar.WristTransfer2);
                leftIn.setPosition(FFVar.InWait);
                rightIn.setPosition(FFVar.InWait);
                Transfer3action = false;

                Transfer4Timer.reset();
                Transfer4action = true;
            }

        }

        if (Transfer4action) {
            if (Transfer4Timer.seconds() >= FFVar.TransferDTime) {

                wrist.setPosition(FFVar.WristOut);
                outArm.setPosition(FFVar.ArmOut);
                leftIn.setPosition(FFVar.InUp);
                rightIn.setPosition(FFVar.InUp);
                Transfer4action = false;

                transfering = false;
                sample = true;
            }
        }
// Handle horizontal slide reset logic
        if (hortouch.isPressed()) {
            resethor = false;
        }
        if (vertouch.isPressed()) {
            resetver = false;
        }
        if (!resethor && !transfering) {
            if (!limitCalculated) {
                int speedBuffer = (int) (horSlide.getPower() * 230); // Buffer proportional to speed (tune the factor)
                dynamicLimit = 1200 - speedBuffer;
                limitCalculated = true; // Lock the limit while the slide is moving
            }
            if (horSlide.getCurrentPosition() > dynamicLimit) {
                if (-gamepad2.right_stick_y < 0) {
                    horSlide.setPower(-gamepad2.right_stick_y);
                } else {
                    horSlide.setPower(0);
                }
            } else {
                horSlide.setPower(-gamepad2.right_stick_y); // Horizontal slide
                if (-gamepad2.right_stick_y != 0 && !hortouch.isPressed() && leftIn.getPosition() < 0.3) {
                    leftIn.setPosition(FFVar.InWait);
                    rightIn.setPosition(FFVar.InWait);
                } else if (hortouch.isPressed()) {
                    leftIn.setPosition(FFVar.InUp);
                    rightIn.setPosition(FFVar.InUp);
                }
            }

        }
        if (Math.abs(-gamepad2.right_stick_y) < 0.1) {
            limitCalculated = false; // Allow recalculation of the limit
        }
// Handle vertical slide reset logic

        if (!resetver && !transfering) {
            if (vertSlideR.getCurrentPosition() > 4000) {
                if (gamepad2.left_stick_y > 0) {
                    vertSlideL.setPower(-gamepad2.left_stick_y); // Vertical slide
                    vertSlideR.setPower(-gamepad2.left_stick_y);
                } else {
                    vertSlideL.setPower(0); // Vertical slide
                    vertSlideR.setPower(0);
                }
            } else {
                vertSlideL.setPower(-gamepad2.left_stick_y); // Vertical slide
                vertSlideR.setPower(-gamepad2.left_stick_y);
            }

        }



        if (gamepad1.dpad_up) {
            FFVar.targetPosition += 100; // Move up by 100 encoder ticks
        } else if (gamepad1.dpad_down) {
            FFVar.targetPosition -= 100; // Move down by 100 encoder ticks
        }

// Clamp target position to safe limits
        FFVar.targetPosition = Math.max(0, Math.min(4000, FFVar.targetPosition)); // Adjust range as needed
    }
    public void Telemetry() {
        telemetry.addData("Vertical Right Slide Pos", vertSlideR.getCurrentPosition());
        telemetry.addData("Vertical left Slide Pos", vertSlideL.getCurrentPosition());
        telemetry.addData("Horizontal Slide Pos", horSlide.getCurrentPosition());
        telemetry.addData("hor power", horSlide.getPower());
        telemetry.addData("red", coloursensor.red());
        telemetry.addData("green", coloursensor.green());
        telemetry.addData("blue", coloursensor.blue());
        telemetry.addData("gamepad x", gamepad1.right_stick_x);
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

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double Yaw= robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll = robotOrientation.getRoll (AngleUnit.DEGREES);
        telemetry.addLine("   ");
        telemetry.addData("Pitch:",Yaw);
        telemetry.addData("Pitch",Pitch);
        telemetry.addData("Roll",Roll);
    }
    @Override
    public void loop () {
        drive();
        manualTake();
        Telemetry();
    }
}