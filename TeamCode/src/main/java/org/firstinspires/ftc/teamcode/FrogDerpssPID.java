package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
@TeleOp(name = "FrogDerpsssPID", group= "TeleOp")
public class FrogDerpssPID extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo leftIn, rightIn, wrist, outArm, claw;
    private DcMotor horSlide, vertSlideL, vertSlideR, intake;



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
    private ElapsedTime runTime = new ElapsedTime();
    boolean Transfer4action = false;

    private ColorSensor coloursensor;

    private TouchSensor hortouch;
    private TouchSensor vertouch;
    boolean resethor = false;
    boolean resetver = false;
    boolean intaking = false;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;



    @Override
    public void init() {

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



        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        hortouch = hardwareMap.get(TouchSensor.class, "hortouch");
        vertouch = hardwareMap.get(TouchSensor.class, "vertouch");
        coloursensor = hardwareMap.get(ColorSensor.class, "coloursensor");
        timer.reset();


        // Blocking delay (not recommended for complex systems)
        while (timer.seconds() < 0.3) {
            // Wait for 1 second
        }
        // Move the intake up
        leftIn.setPosition(FFVar.InUp);
        rightIn.setPosition(FFVar.InUp);

    }


    public void drive() {
        // Input scaling
        double driveX = scaleInput(gamepad1.left_stick_x);
        double driveY = scaleInput(-gamepad1.left_stick_y); // Invert Y-axis
        double RightTurn = gamepad1.right_trigger;
        double LeftTurn = gamepad1.left_trigger;

        // Dynamic turning adjustment
        double turnvar = Math.max(1, 1 + (horSlide.getCurrentPosition() / 1000.0));
        double rotate = (0.7 * (RightTurn - LeftTurn)) / turnvar;

        // Mecanum drive calculations
        double angle = Math.atan2(driveY, driveX) - Math.PI / 4;
        double magnitude = Math.min(1.0, Math.hypot(driveX, driveY) - 0.1);

        double frontLeftPower = magnitude * Math.cos(angle) + rotate;
        double frontRightPower = magnitude * Math.sin(angle) - rotate;
        double backLeftPower = magnitude * Math.sin(angle) + rotate;
        double backRightPower = magnitude * Math.cos(angle) - rotate;

        // Normalize motor powers
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private double scaleInput(double input) {
        double threshold = 0.6;
        double slowFactor = 0.3;

        if (Math.abs(input) > threshold) {
            return input; // Full speed
        } else {
            return input * slowFactor; // Reduced speed
        }
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


        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) { //Intake
            if (intake.getPower() < 0.2) {
                intake.setPower(0.8);
                intaking = true;
            } else if (intake.getPower() > 0.2 && currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intake.setPower(0);
                intaking = false;
            }
        }

        if (intaking && red > green && red > blue && red > 50) { //detects red
            intake.setPower(0);
            intaking = false;
        }
        if (intaking && blue > red && blue > green && blue > 50) { //detects blue
            intake.setPower(0);
            intaking = false;
        }


        if (intaking && red > 50 && green > 50 && blue < 30) { //detects yellow
            intake.setPower(0);
            intaking = false;
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            if (intake.getPower() > -0.2) {
                intake.setPower(-0.6); // Reverse
            } else {
                intake.setPower(0); // Stop
            }
        }
        if (currentGamepad1.cross && !previousGamepad1.cross) { // Arm down
            if (leftIn.getPosition() > 0.5) {
                // Set servo positions to ArmDwn
                leftIn.setPosition(FFVar.InWait);
                rightIn.setPosition(FFVar.InWait);
            } else if ((leftIn.getPosition() < 0.5) && Math.abs(horSlide.getCurrentPosition()) > 300) {
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


        if (currentGamepad1.circle && !previousGamepad1.circle) {
            // Start the Outtake action
            outArm.setPosition(FFVar.ArmOut);
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
            }
        }
        if (currentGamepad1.square && !previousGamepad1.square) {
            if (!hortouch.isPressed()) {
                horSlide.setPower(-1);
            }
            if (!vertouch.isPressed()) {
                vertSlideR.setPower(-0.8);
                vertSlideL.setPower(-0.8);
            }
            // Move the arm and wrist to their positions
            outArm.setPosition(FFVar.ArmTransfer);
            wrist.setPosition(FFVar.WristTransfer);
            claw.setPosition(FFVar.ClawOpen);
            intake.setPower(0);
            leftIn.setPosition(FFVar.InWait);
            rightIn.setPosition(FFVar.InWait);

            // Initialize reset flags
            resethor = true;
            resetver = true;

            // Reset the timer and set actionInProgress
            Transfer1Timer.reset();
            Transfer1action = true; // Start a new action sequence
        }

// Check if an action is in progress
        if (Transfer1action) {
            // Wait for 2 seconds using non-blocking timer
            if (Transfer1Timer.seconds() >= FFVar.TransferATime + (vertSlideL.getCurrentPosition()/400) + (horSlide.getCurrentPosition()/900)) {
                // Set the positions for leftIn and rightIn after delay
                leftIn.setPosition(FFVar.InTransfer);
                rightIn.setPosition(FFVar.InTransfer);

                Transfer2action = true;
                Transfer2Timer.reset();

                // End the action
                Transfer1action = false;
            }
        }
        if (Transfer2action) {
            // Wait for 2 seconds using non-blocking timer
            if (Transfer2Timer.seconds() >= FFVar.TransferBTime) {
                // Set the positions for leftIn and rightIn after delay
                claw.setPosition(FFVar.ClawClose);

                Transfer3action = true;
                Transfer3Timer.reset();

                // End the action
                Transfer2action = false;
            }
        }
        if (Transfer3action) {
            if (Transfer3Timer.seconds() >= FFVar.TransferCTime) {

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
                outArm.setPosition(FFVar.ArmWait);
                wrist.setPosition(FFVar.WristWait);
                Transfer4action = false;
            }
        }
// Handle horizontal slide reset logic
        if (hortouch.isPressed()) {
            resethor = false;
        }
        if (!resethor) {
            horSlide.setPower(gamepad1.right_stick_x); // Horizontal slide
        }

// Handle vertical slide reset logic
        if (vertouch.isPressed()) {
            resetver = false;
        }
        if (!resetver) {
            vertSlideL.setPower(-gamepad1.right_stick_y); // Vertical slide
            vertSlideR.setPower(-gamepad1.right_stick_y);
        }



        if (gamepad1.dpad_up) {
            FFVar.targetPosition += 100; // Move up by 100 encoder ticks
        } else if (gamepad1.dpad_down) {
            FFVar.targetPosition -= 100; // Move down by 100 encoder ticks
        }

// Clamp target position to safe limits
        FFVar.targetPosition = Math.max(0, Math.min(4000, FFVar.targetPosition)); // Adjust range as needed


        double currentPosition= vertSlideL.getCurrentPosition();
        double error = (float) (FFVar.targetPosition-currentPosition);
        FFVar.integralSum += (float) (error * runTime.seconds());
        double derivative = (error-FFVar.lastError)/runTime.seconds();
        double power = FFVar.kP*error+FFVar.kI*FFVar.integralSum+FFVar.kD*derivative;
        vertSlideL.setPower(power);
        runTime.reset();
        FFVar.lastError= (float) error;
    }
    public void Telemetry() {
        telemetry.addData("Vertical Right Slide Pos", vertSlideR.getCurrentPosition());
        telemetry.addData("Vertical left Slide Pos", vertSlideL.getCurrentPosition());
        telemetry.addData("Horizontal Slide Pos", horSlide.getCurrentPosition());
        telemetry.addData("hor power", horSlide.getPower());
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
    }
    @Override
    public void loop () {
        drive();
        manualTake();
        Telemetry();
    }
}