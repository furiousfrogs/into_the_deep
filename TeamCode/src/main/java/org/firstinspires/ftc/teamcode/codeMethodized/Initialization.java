package org.firstinspires.ftc.teamcode.codeMethodized;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Initialization {
    public DcMotor frontLeft,frontRight,backLeft,backRight;
    public Servo leftIn, rightIn, wrist, outArm, claw, gate;
    public DcMotor horSlide,vertSlideL, vertSlideR, intake;
    public ColorRangeSensor coloursensor;
    public TouchSensor hortouch;
    public TouchSensor vertouch;
    //variables
    double[] currentGod = VarAll.initPos;
    double[] previousGod = currentGod;
    boolean target=false; //false is specimen, true is sample
    boolean color=false;  //false is blue, true is red
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad2;

    public void initialize(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        //initalized stuff
        frontLeft=hardwareMap.get(DcMotor.class,"front_left");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight=hardwareMap.get(DcMotor.class,"front_right");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft=hardwareMap.get(DcMotor.class,"back_left");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight=hardwareMap.get(DcMotor.class,"back_right");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftIn=hardwareMap.get(Servo.class,"leftIn");
        leftIn.setDirection(Servo.Direction.FORWARD);

        rightIn=hardwareMap.get(Servo.class,"rightIn");
        rightIn.setDirection(Servo.Direction.FORWARD);


        outArm=hardwareMap.get(Servo.class,"outArm");
        outArm.setDirection(Servo.Direction.FORWARD);

        wrist=hardwareMap.get(Servo.class,"wrist");
        wrist.setDirection(Servo.Direction.FORWARD);

        claw=hardwareMap.get(Servo.class,"claw");
        claw.setDirection(Servo.Direction.FORWARD);


        horSlide=hardwareMap.get(DcMotor.class,"rightHor");
        horSlide.setDirection(DcMotor.Direction.REVERSE);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horSlide=hardwareMap.get(DcMotor.class,"rightHor");
        horSlide.setDirection(DcMotor.Direction.FORWARD);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideL=hardwareMap.get(DcMotor.class,"leftVertical");
        vertSlideL.setDirection(DcMotor.Direction.REVERSE);
        vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlideR=hardwareMap.get(DcMotor.class,"rightVertical");
        vertSlideR.setDirection(DcMotor.Direction.REVERSE);
        vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake=hardwareMap.get(DcMotor.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate=hardwareMap.get(Servo.class,"gate");
        gate.setDirection(Servo.Direction.FORWARD);

        currentGamepad1 =gamepad1;
        previousGamepad1.copy(currentGamepad1);
        currentGamepad2 =gamepad2;
        previousGamepad2.copy(currentGamepad2);


        hortouch=hardwareMap.get(TouchSensor.class,"horTouch");
        vertouch=hardwareMap.get(TouchSensor.class,"verTouch");
        coloursensor=hardwareMap.get(ColorRangeSensor.class,"colourSensor");
    }
}
