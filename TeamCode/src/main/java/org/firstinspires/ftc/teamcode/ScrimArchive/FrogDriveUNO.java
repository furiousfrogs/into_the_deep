package org.firstinspires.ftc.teamcode.ScrimArchive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp(name = "FrogDriveUNO", group= "TeleOp")
public class FrogDriveUNO extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo arm, wrist, claw;
    private Servo servoL, servoR;
    private DcMotor spin;
    private DcMotor horSlide, vertSlideL, vertSlideR;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    private static final float ArmUp = 0.5F; //TUNE!!!!!!!!!!!!!!!!!!!!
    private static final float ArmDwn = 0.06F; //TUNE!!!!!!!!!!!!!!!!!!!!!!!!!
    private static final float Outtake = 0.0F; //TUNE!!!!!!!!!!!!!!!!!!!!
    private static final float Outtake_Intake = 0.65F;
    private int horPos = 0;
    private static final int horDesiredPos = 500;
    private static final int vertPos1 = 0;
    private static final int vertPos2 = 1000;
    boolean SlowMode = false;
    boolean buttonIsReleased = true;
    private TouchSensor hortouch;
    private TouchSensor vertouch;
    boolean resethor = false;
    boolean resetver = false;

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

        servoL = hardwareMap.get(Servo.class, "leftarm");
        servoL.setDirection(Servo.Direction.FORWARD);
        servoL.setPosition(0);

        servoR = hardwareMap.get(Servo.class, "rightarm");
        servoR.setDirection(Servo.Direction.FORWARD);
        servoR.setPosition(0);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(0);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(0);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(0);


        horSlide = hardwareMap.get(DcMotor.class, "righthor");
        horSlide.setDirection(DcMotor.Direction.REVERSE);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideL = hardwareMap.get(DcMotor.class, "leftvertical");
        vertSlideL.setDirection(DcMotor.Direction.REVERSE);
        vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideR = hardwareMap.get(DcMotor.class, "rightvertical");
        vertSlideR.setDirection(DcMotor.Direction.FORWARD);
        vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spin =hardwareMap.get(DcMotor.class,"spin");
        spin.setDirection(DcMotor.Direction.FORWARD);
        spin.setPower(0);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

    }
    public void drive() {

        double driveX = 0;
        double driveY = 0;
        double RightTurn = gamepad1.right_trigger;
        double LeftTurn = gamepad1.left_trigger;
        double slowvar = 0.5;
        if (Math.abs(gamepad1.left_stick_x) > 0.4) {
            driveX = gamepad1.left_stick_x;
        } else if (Math.abs(gamepad1.left_stick_x) <= 0.4) {
            driveX = slowvar * gamepad1.left_stick_x;
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.4) {
            driveY = -gamepad1.left_stick_y;

        } else if (Math.abs(gamepad1.left_stick_y) <= 0.4) {
            driveY = slowvar * -gamepad1.left_stick_y;
        }
        double rotate;
        if (horSlide.getCurrentPosition()>100){
            rotate = 0.2*(RightTurn-LeftTurn);
        }
        else{
            rotate=0.7*(RightTurn-LeftTurn);
        }
        double angle = Math.atan2(driveY, driveX) - Math.PI / 4;//caculate output
        double magnitude = Math.hypot(driveX, driveY) - 0.1;//MIGHT BREAK CODE IDK CONSTANT VALUE
        double frontLeftPower = magnitude * Math.cos(angle) + rotate;
        double frontRightPower = magnitude * Math.sin(angle) - rotate;
        double backLeftPower = magnitude * Math.sin(angle) + rotate;
        double backRightPower = magnitude * Math.cos(angle) - rotate;
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        frontLeft.setPower(frontLeftPower);//giving output
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    public void manualTake() {
        //manual controls

        // Store previous state
        previousGamepad1.copy(currentGamepad1);
        // Update current state with the latest gamepad data
        currentGamepad1.copy(gamepad1);


        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) { //Intake
            if (spin.getPower() == 0) {
                spin.setPower(-1);
            } else {
                spin.setPower(0);
            }

        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) { //Outtake
            if (spin.getPower() == 0) {
                spin.setPower(1);
            } else {
                spin.setPower(0);
            }
        }
        if (currentGamepad1.cross && !previousGamepad1.cross) { // Arm down
            if (servoL.getPosition() > 0.2 && Math.abs(horSlide.getCurrentPosition()) > 50) {
                // Set servo positions to ArmDwn
                servoL.setPosition(ArmDwn);
                servoR.setPosition(ArmDwn);
            } else if (Math.abs(horSlide.getCurrentPosition()) < 50) {
                // Move horSlide until it reaches position 50
                do {
                    horSlide.setPower(0.5);
                } while (horSlide.getCurrentPosition() > 50);
            } else {
                // Set servo positions to ArmUp
                servoL.setPosition(ArmUp);
                servoR.setPosition(ArmUp);
            }
        }


        if (currentGamepad1.square && !previousGamepad1.square) {
            spin.setPower(0);
            servoL.setPosition(ArmUp);
            servoR.setPosition(ArmUp);
            resethor = true;
            resetver = true;
            if (!hortouch.isPressed()) {
                horSlide.setPower(-1);
            }
            if (!vertouch.isPressed()) {
                vertSlideR.setPower(-0.8);
                vertSlideL.setPower(-0.8);
            }
        } else if (hortouch.isPressed()) {
            resethor = false;
        } else if (vertouch.isPressed()) {
            resetver = false;
        }
        if (!resethor) {
            horSlide.setPower(gamepad1.right_stick_x); //Horizontalslide
        }
        if (!resetver) {
            vertSlideL.setPower(-gamepad2.right_stick_y-gamepad1.right_stick_y); //Vertical Slide
            vertSlideR.setPower(-gamepad2.right_stick_y-gamepad1.right_stick_y);
        }

    }
    public void Telemetry() {
        telemetry.addData("Vertical Right Slide Pos", vertSlideR.getCurrentPosition());
        telemetry.addData("Vertical left Slide Pos", vertSlideL.getCurrentPosition());
        telemetry.addData("Horizontal Slide Pos", horSlide.getCurrentPosition());
    }
    @Override
    public void loop () {
        drive();
        manualTake();
        Telemetry();
//        if (mode ==1){
//            try {
//                programmedTake();
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
//        else if (mode == 0){
//            manualTake();
//        }
    }
}












