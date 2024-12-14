package org.firstinspires.ftc.teamcode;

// RR-specific imports
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Arrays;


@Config
@Autonomous(name = "frogtonomousweird", group = "Autonomous")
public class frogtonomousred extends LinearOpMode{

    public class push {
        private TouchSensor hortouch;
        private DcMotor spin;
        private Servo leftintake, rightintake, armwrist, arm;
        private DcMotor horizontalslide;
        private ElapsedTime timer = new ElapsedTime();

        public push(HardwareMap hardwareMap){
            horizontalslide = hardwareMap.get(DcMotor.class, "righthor");
            horizontalslide.setDirection(DcMotor.Direction.REVERSE);
            horizontalslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            horizontalslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            hortouch = hardwareMap.get(TouchSensor.class, "hortouch");

            leftintake = hardwareMap.get(Servo.class, "leftin");
            leftintake.setDirection(Servo.Direction.FORWARD);
            leftintake.setPosition(FFVar.InUp);

            rightintake = hardwareMap.get(Servo.class, "rightin");
            rightintake.setDirection(Servo.Direction.FORWARD);
            rightintake.setPosition(FFVar.InUp);

            spin = hardwareMap.get(DcMotor.class, "intake");
            spin.setDirection(DcMotor.Direction.FORWARD);
            spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //not relevant
            armwrist = hardwareMap.get(Servo.class, "wrist");
            armwrist.setDirection(Servo.Direction.FORWARD);
            armwrist.setPosition(FFVar.WristOut);

            arm = hardwareMap.get(Servo.class, "outarm");
            arm.setDirection(Servo.Direction.FORWARD);
            arm.setPosition(FFVar.ArmInit);
        }

        public class pushset implements Action{

            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    horizontalslide.setPower(0.7);
                    initialized = true;
                }

                // checks lift's current position
                double pos = horizontalslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 400.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    horizontalslide.setPower(0);
                    leftintake.setPosition(FFVar.InDown);
                    rightintake.setPosition(FFVar.InDown);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action pushsetup() {
            return new pushset();
        }

        public class pushouts implements Action{

            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    spin.setPower(1);
                    timer.reset();
                    initialized = true;
                }

                // checks lift's current position
                double pos = horizontalslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (timer.seconds() < 0.5) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    spin.setPower(-1);
                    timer.reset();
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action pushout() {
            return new pushouts();
        }

        public class Pushforward implements Action {
            private boolean initialized = false;




            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    horizontalslide.setPower(0.7);
                    spin.setPower(-1);
                    initialized = true;
                }


                double pos = horizontalslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 980.0) {
                    return true;
                } else {
                    horizontalslide.setPower(0);
                    return false;
                }
            }
        }

        public Action pushtake() {return new Pushforward();}
        public class PushDown implements Action {
            private boolean initialized = false;



            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    horizontalslide.setPower(-1);
                    leftintake.setPosition(FFVar.InUp);
                    rightintake.setPosition(FFVar.InUp);
                    spin.setPower(0);
                    initialized = true;
                }

                if (!hortouch.isPressed()) {
                    return true;
                } else if (hortouch.isPressed()) {
                    horizontalslide.setPower(0);
                    return false;
                }
                return false;
            }
        }

        public Action pushreturn(){
            return new PushDown();
        }
    }

    public class lift {
        private TouchSensor vertouch;
        DcMotor verticalslideL, verticalslideR;

        public lift(HardwareMap hardwareMap){
            verticalslideL = hardwareMap.get(DcMotor.class, "leftvertical");
            verticalslideL.setDirection(DcMotor.Direction.FORWARD);
            verticalslideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalslideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalslideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalslideR = hardwareMap.get(DcMotor.class, "rightvertical");
            verticalslideR.setDirection(DcMotor.Direction.REVERSE);
            verticalslideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalslideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalslideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            vertouch = hardwareMap.get(TouchSensor.class, "vertouch");
        }

        public class liftblock implements Action{

            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    verticalslideL.setPower(1);
                    verticalslideR.setPower(1);
                    initialized = true;
                }

                // checks lift's current position
                double pos1 = verticalslideL.getCurrentPosition();
                double pos2 = verticalslideR.getCurrentPosition();
                double pos = (pos1+pos2)/2;
                packet.put("liftPos", pos);
                if (pos < 1950.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    verticalslideL.setPower(0);
                    verticalslideR.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action liftup() {
            return new liftblock();
        }
        public class liftwall implements Action{

            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    verticalslideL.setPower(1);
                    verticalslideR.setPower(1);
                    initialized = true;
                }

                // checks lift's current position
                double pos1 = verticalslideL.getCurrentPosition();
                double pos2 = verticalslideR.getCurrentPosition();
                double pos = (pos1+pos2)/2;
                packet.put("liftPos", pos);
                if (pos < 500.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    verticalslideL.setPower(0);
                    verticalslideR.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftfromwall(){
            return new liftwall();
        }
        public class liftreturn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    verticalslideL.setPower(-1);
                    verticalslideR.setPower(-1);
                    initialized = true;
                }

                if (!vertouch.isPressed()) {
                    return true;
                } else if (vertouch.isPressed()) {
                    verticalslideR.setPower(0);
                    verticalslideL.setPower(0);
                    verticalslideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    verticalslideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    verticalslideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    verticalslideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    ;                    return false;
                }
                return false;
            }
        }
        public Action liftback(){
            return new liftreturn();
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
        lift LIFTFROGGY = new lift(hardwareMap);

        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(-8, 62, Math.toRadians(90)));





        TrajectoryActionBuilder blueside1 = myBot.actionBuilder(new Pose2d(-8, 62, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-2, 36));
        TrajectoryActionBuilder blueside2 = myBot.actionBuilder(new Pose2d(-2, 36, Math.toRadians(90)))
                .lineToY(30);
        TrajectoryActionBuilder blueside3 = myBot.actionBuilder(new Pose2d(-8, 32, Math.toRadians(90)))
                .splineTo(new Vector2d(-34, 40), Math.toRadians(223));
        TrajectoryActionBuilder blueside4 = myBot.actionBuilder(new Pose2d(-34, 40, Math.toRadians(223)))
                .turnTo(Math.toRadians(120));
        TrajectoryActionBuilder blueside5 = myBot.actionBuilder(new Pose2d(-30, 40, Math.toRadians(135)))
                .splineTo(new Vector2d(-39, 40), Math.toRadians(216));
        TrajectoryActionBuilder blueside6 = myBot.actionBuilder(new Pose2d(-39, 40, Math.toRadians(216)))
                .turnTo(Math.toRadians(120));
        TrajectoryActionBuilder blueside7 = myBot.actionBuilder(new Pose2d(-44, 40,Math.toRadians(135)))
                .strafeToSplineHeading(new Vector2d(-40, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(67);
        TrajectoryActionBuilder blueside8 = myBot.actionBuilder(new Pose2d(-40, 67, Math.toRadians(270)))
                .lineToYConstantHeading(50)
                .strafeToSplineHeading(new Vector2d(-4, 36), Math.toRadians(90) );
        TrajectoryActionBuilder blueside9 = myBot.actionBuilder(new Pose2d(-4, 36, Math.toRadians(90)))
                .lineToYConstantHeading(28);
        TrajectoryActionBuilder blueside10 = myBot.actionBuilder(new Pose2d(-4, 28, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-40, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(69);
        TrajectoryActionBuilder blueside11 = myBot.actionBuilder(new Pose2d(-40, 66, Math.toRadians(270)))
                .lineToYConstantHeading(50)
                .strafeToSplineHeading(new Vector2d(4, 36), Math.toRadians(90));
        TrajectoryActionBuilder blueside12 = myBot.actionBuilder(new Pose2d(4, 36, Math.toRadians(90)))
                .lineToYConstantHeading(31);
        TrajectoryActionBuilder blueside13 = myBot.actionBuilder(new Pose2d(4, 28, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-40, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(66);
//        TrajectoryActionBuilder blueside14 = myBot.actionBuilder(new Pose2d(-40, 65, Math.toRadians(270)))
//                .lineToYConstantHeading(50)
//                .strafeToSplineHeading(new Vector2d(3, 36), Math.toRadians(90));
//        TrajectoryActionBuilder blueside15 = myBot.actionBuilder(new Pose2d(3, 36, Math.toRadians(90)))
//                .lineToYConstantHeading(33);


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Action traj1 = null;
        Action traj2 = null;
        Action traj3 = null;
        Action traj4 = null;
        Action traj5 = null;
        Action traj6 = null;
        Action traj7 = null;
        Action traj8 = null;
        Action traj9 = null;
        Action traj10 = null;
        Action traj11 = null;
        Action traj12 = null;
        Action traj13 = null;
        Action traj14 = null;
        Action traj15 = null;


        traj1 = blueside1.build();
        traj2 = blueside2.build();
        traj3 = blueside3.build();
        traj4 = blueside4.build();
        traj5 = blueside5.build();
        traj6 = blueside6.build();
        traj7 = blueside7.build();
        traj8 = blueside8.build();
        traj9 = blueside9.build();
        traj10 = blueside10.build();
        traj11 = blueside11.build();
        traj12 = blueside12.build();
        traj13 = blueside13.build();
//        traj14 = blueside14.build();
//        traj15 = blueside15.build();

        //new SleepAction(1),

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                traj1,
                                new SequentialAction(
                                        LIFTFROGGY.liftup()
                                )
                        ),
                        traj2,
                        LIFTFROGGY.liftback(),

                        PUSHFROGGY.pushsetup(),
                        //2spec? here
                        traj3,
                        PUSHFROGGY.pushtake(),
                        traj4,
                        PUSHFROGGY.pushout(),
                        traj5,
                        traj6,
                        PUSHFROGGY.pushout(),
                        PUSHFROGGY.pushreturn(),
                        traj7,
                        LIFTFROGGY.liftfromwall(),
                        new ParallelAction(
                                traj8,
                                new SequentialAction(
                                        LIFTFROGGY.liftup()
                                )
                        ),
                        traj9,
                        LIFTFROGGY.liftback(),
                        traj10,
                        LIFTFROGGY.liftfromwall() ,
                        new ParallelAction(
                                traj11,
                                new SequentialAction(
                                        LIFTFROGGY.liftup()
                                )
                        ),
                        traj12,
                        LIFTFROGGY.liftback(),
                        traj13
//                        LIFTFROGGY.liftfromwall(),
//                        traj14,
//                        LIFTFROGGY.liftup(),
//                        traj15,
//                        LIFTFROGGY.liftback()

                )
        );

    }

}