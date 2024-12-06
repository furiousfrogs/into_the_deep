package org.firstinspires.ftc.teamcode;

// RR-specific imports
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Config
@Autonomous(name = "frogauto", group = "Autonomous")
public class frogautonomous extends LinearOpMode{


//
//    TouchSensor vertouch;
//    int side = 0;//0 is blue side, 1 is red side
//    DcMotor horizontalslide, verticalslideL, verticalslideR;


    public class push {
        private TouchSensor hortouch;
        private DcMotor horizontalslide;

        public push(HardwareMap hardwareMap){
            horizontalslide = hardwareMap.get(DcMotor.class, "righthor");
            horizontalslide.setDirection(DcMotor.Direction.REVERSE);
            horizontalslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            horizontalslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hortouch = hardwareMap.get(TouchSensor.class, "hortouch");
        }

        public class pushblock implements Action{

            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    horizontalslide.setPower(0.5);
                    initialized = true;
                }

                // checks lift's current position
                double pos = horizontalslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1500.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    horizontalslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action pushf() {
            return new pushblock();
        }
        public class PushDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    horizontalslide.setPower(-0.5);
                    initialized = true;
                }

                double pos = horizontalslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else if (hortouch.isPressed()) {
                    horizontalslide.setPower(0);
                    return false;
                }
                return false;
            }
        }
        public Action pushb(){
            return new PushDown();
        }
    }
//

//    @Override
//    public void init() throws InterruptedException {
//
//        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(-10, 62, Math.toRadians(270)));
//
//        horizontalslide = hardwareMap.get(DcMotor.class, "righthor");
//        horizontalslide.setDirection(DcMotor.Direction.REVERSE);
//        horizontalslide .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontalslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        horizontalslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        verticalslideL = hardwareMap.get(DcMotor.class, "leftvertical");
//        verticalslideL.setDirection(DcMotor.Direction.REVERSE);
//        verticalslideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalslideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        verticalslideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        verticalslideR = hardwareMap.get(DcMotor.class, "rightvertical");
//        verticalslideR.setDirection(DcMotor.Direction.FORWARD);
//        verticalslideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalslideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        verticalslideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        hortouch = hardwareMap.get(TouchSensor.class, "hortouch");
//        vertouch = hardwareMap.get(TouchSensor.class, "vertouch");
//
//        if (hortouch.isPressed()) { //Horizontal touch sensor detection
//
//            horizontalslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            horizontalslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//
//        if (vertouch.isPressed()) { //Reset vertical encoders
//            verticalslideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalslideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            verticalslideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalslideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//        waitForStart();
//    }


    @Override
    public void runOpMode() {//specimen hang 1.5 to 2.5k pos
        int side = 0;
        push PUSHFROGGY = new push(hardwareMap);

        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(-10, 62, Math.toRadians(270)));

        //Action Action1 = intakepush();
        //Action Action2 = specimenhang();

        TrajectoryActionBuilder blueside1 = myBot.actionBuilder(new Pose2d(-10, 62, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(-25, 60))
                .waitSeconds(1)
                .splineTo(new Vector2d(-48, 15), Math.toRadians(90))
                .waitSeconds(1);
        //move horizontal slide
//                    .strafeToConstantHeading(new Vector2d(-57, 15))
//                    .waitSeconds(1)
//                    //.afterDisp(40, Action1)
//                    //move horizontal slide;
//                    .splineTo(new Vector2d(-35, 50), Math.toRadians(270))
//                    .lineToYConstantHeading(62)
//                    //just repeat 5 times but cannot use for loop :(
//                    .lineToYConstantHeading(50)
//                    .splineTo(new Vector2d(-6, 50), Math.toRadians(90))
//                    .lineToYConstantHeading(32);
        //.afterDisp(60, Action2);

        //move vertical slide


        TrajectoryActionBuilder redside = myBot.actionBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(25, -60))
                .waitSeconds(0.000001)
                .splineTo(new Vector2d(48, -15), Math.toRadians(270));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = null;
        if (side == 0) {
            trajectoryActionChosen = blueside1.build();
        } else if (side == 1) {
            redside.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        PUSHFROGGY.pushf(),
                        PUSHFROGGY.pushb()


                )
        );

    }

}