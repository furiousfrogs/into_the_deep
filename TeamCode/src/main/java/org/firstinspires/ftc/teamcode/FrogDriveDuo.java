package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "FrogDriveDuo", group= "TeleOp")
public class FrogDriveDuo extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo servoL, servoR, spinOut;
    private CRServo spinIn;
    private DcMotor horSlide, vertSlideL, vertSlideR;
    private int step = 0;
    //0: vert and hor retracted, inouttake not spinning, servos are tucked in
    //1: hor is retracted, vert is slowly extending, intake spinning, outtake not spinning, servos turned out
    //2: hor and vert is retracted, intake spinning opposite, so object transfered to outtake, servos turned in
    //3: vert is extended, outtake is spinnning
    //private int mode = 1;
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
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;


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
        servoL.setDirection(Servo.Direction.REVERSE);
        servoL.setPosition(ArmUp);

        servoR = hardwareMap.get(Servo.class, "rightarm");
        servoR.setDirection(Servo.Direction.FORWARD);
        servoR.setPosition(ArmUp);

        spinIn = hardwareMap.get(CRServo.class, "intake");
        spinOut = hardwareMap.get(Servo.class, "outtake");
        spinOut.setPosition(Outtake_Intake);

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

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();
        hortouch = hardwareMap.get(TouchSensor.class, "hortouch");
        vertouch = hardwareMap.get(TouchSensor.class, "vertouch");


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


        double rotate = 0.7 * (RightTurn - LeftTurn);
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
        telemetry.addData("frontLeftPower",frontLeft.getPower());
        telemetry.addData("front right",frontRight.getPower());
        telemetry.addData("backleft",backLeft.getPower());
        telemetry.addData("backright",backRight.getPower());

    }

    //    public void programmedTake() throws InterruptedException {
//        if (gamepad2.triangle){
//            step+=1;
//        }
//        if (step>4){
//            step=0;
//        }
//        if (step==0){
//            horPos=0;
//            horSlide.setTargetPosition(horPos);
//            vertSlideL.setTargetPosition(vertPos1);
//            vertSlideR.setTargetPosition(vertPos1);
//            spinIn.setPower(0);
//            spinOut.setPower(0);
//            servoL.setPosition(ArmUp);
//            servoR.setPosition(ArmUp);
//        }
//        else if (step==1){
//            servoL.setPosition(ArmDwn);
//            servoR.setPosition(ArmDwn);
//            spinIn.setPower(1);
//            horPos+=10;
//            horSlide.setTargetPosition(horPos);
//        }
//        else if (step==2) {
//            servoL.setPosition(ArmUp);
//            servoR.setPosition(ArmUp);
//            spinIn.setPower(0);
//            horPos = 0;
//            horSlide.setTargetPosition(horPos);
//        }
//        else if (step==3){
//            spinIn.setPower(-1);
//        }
//        else if (step ==4){
//            spinIn.setPower(0);
//            vertSlideL.setTargetPosition(vertPos2);
//            vertSlideR.setTargetPosition(vertPos2);
//            spinOut.setPower(-1);
//        }
//        if (gamepad2.square) {
//            step = 0;
//        }
//        if (gamepad2.circle){
//            vertSlideL.setTargetPosition(vertPos2);
//            vertSlideR.setTargetPosition(vertPos2);
//        }
//        if (gamepad2.left_bumper){
//            mode=0;
//        }
//    }
    //programmed contorls
    public void manualTake() {
        //manual controls

        // Store previous state
        previousGamepad1.copy(currentGamepad1);
        // Update current state with the latest gamepad data
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        // Update current state with the latest gamepad data
        currentGamepad2.copy(gamepad2);

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) { //Intake
            if (spinIn.getPower() == 0) {
                spinIn.setPower(-1);
            } else {
                spinIn.setPower(0);
            }

        } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) { //Outtake
            if (spinIn.getPower() == 0) {
                spinIn.setPower(1);
            } else {
                spinIn.setPower(0);
            }
        }


//        horSlide.setPower(gamepad1.right_stick_x); //Horizontalslide
//
//
//
//        vertSlideL.setPower(-gamepad1.right_stick_y); //Vertical Slide
//        vertSlideR.setPower(-gamepad1.right_stick_y);


        if (currentGamepad2.cross && !previousGamepad2.cross) { // Arm down
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

        if (hortouch.isPressed()) { //Horizontal touch sensor detection

            horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (vertouch.isPressed()) { //Reset vertical encoders
            vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

//        if (currentGamepad1.square && !previousGamepad1.square) { // Intake reset
//            spinIn.setPower(0);
//            servoL.setPosition(ArmUp);
//            servoR.setPosition(ArmUp);
//            resethor = true;
//            resetver = true;
//
//        }
//
//        if (resethor) {
//            horSlide.setPower(-0.7);
//            if (hortouch.isPressed()) { // Stop horizontal slide when touch sensor is pressed
//                horSlide.setPower(0);
//                 // Reset horizontal flag
//            } else {
//                resethor = false;
//            }
//        }
//
//        if (resetver) {
//            vertSlideR.setPower(-0.8);
//            vertSlideL.setPower(-0.8);
//            if (vertouch.isPressed()) { // Stop vertical slides when touch sensor is pressed
//                vertSlideR.setPower(0);
//                vertSlideL.setPower(0);
//                // Reset vertical flag
//            } else {
//                resetver = false;
//            }
//        }

        if (currentGamepad2.square && !previousGamepad2.square || currentGamepad1.square && !previousGamepad1.square) {
            spinIn.setPower(0);
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
            horSlide.setPower(gamepad2.right_stick_x); //Horizontalslide
        }
        if (!resetver) {
            vertSlideL.setPower(-gamepad2.right_stick_y); //Vertical Slide
            vertSlideR.setPower(-gamepad2.right_stick_y);
        }

//            vertSlideL.setTargetPosition(0);
//            vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            vertSlideL.setPower(-1);
//            vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            vertSlideR.setTargetPosition(0);
//            vertSlideR.setPower(-1);
//            vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (currentGamepad1.circle && !previousGamepad1.circle) { //outtake
            if (spinOut.getPosition() > 0.25) {
                spinOut.setPosition(Outtake);
            } else {
                spinOut.setPosition(Outtake_Intake);
            }
        }

        if (hortouch.isPressed()) {
            spinOut.setPosition(Outtake_Intake);
        }

    }
    public void Telemetry() {
        telemetry.addData("Vertical Right Slide Pos", vertSlideR.getCurrentPosition());
        telemetry.addData("Vertical left Slide Pos", vertSlideL.getCurrentPosition());
        telemetry.addData("Horizontal Slide Pos", horSlide.getCurrentPosition());
        if (hortouch.isPressed()) {
            telemetry.addLine("touch joe");
        }
        if (vertouch.isPressed()) {
            telemetry.addLine("touch chuck");
        }
        if (servoL.getPosition() == ArmUp) {
            telemetry.addLine("Arm Up");
        } else if (servoL.getPosition() == ArmDwn){
            telemetry.addLine("Arm Down");
        }
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