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
@Autonomous(name = "frogtonomous", group = "Autonomous")
public class frogtonomous extends LinearOpMode{
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
        public Action pushforward() {
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
            verticalslideL.setDirection(DcMotor.Direction.REVERSE);
            verticalslideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalslideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalslideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalslideR = hardwareMap.get(DcMotor.class, "rightvertical");
            verticalslideR.setDirection(DcMotor.Direction.FORWARD);
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
                    verticalslideL.setPower(0.5);
                    verticalslideR.setPower(0.5);
                    initialized = true;
                }

                // checks lift's current position
                double pos1 = verticalslideL.getCurrentPosition();
                double pos2 = verticalslideR.getCurrentPosition();
                double pos = (pos1+pos2)/2;
                packet.put("liftPos", pos);
                if (pos < 2500.0) {
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
        public class liftreturn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    verticalslideL.setPower(-0.5);
                    verticalslideR.setPower(-0.5);
                    initialized = true;
                }

                if (!vertouch.isPressed()) {
                    return true;
                } else if (vertouch.isPressed()) {
                    verticalslideR.setPower(0);
                    verticalslideL.setPower(0);
                    return false;
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
        Pose2d initialpose = new Pose2d(-8, 62, Math.toRadians(90));
        push PUSHFROGGY = new push(hardwareMap);
        lift LIFTFROGGY = new lift(hardwareMap);

        MecanumDrive myBot = new MecanumDrive(hardwareMap, initialpose);

        TrajectoryActionBuilder blueside1 = myBot.actionBuilder(initialpose)
                .lineToY(33);
        TrajectoryActionBuilder blueside2 = blueside1.endTrajectory().fresh()
                .splineTo(new Vector2d(-35, 40), Math.toRadians(225));
        TrajectoryActionBuilder blueside3 = blueside2.endTrajectory().fresh()
                .turnTo(Math.toRadians(135));
        TrajectoryActionBuilder blueside4 = blueside3.endTrajectory().fresh()
                .splineTo(new Vector2d(-44, 40), Math.toRadians(225));
        TrajectoryActionBuilder blueside5 = blueside4.endTrajectory().fresh()
                .turnTo(Math.toRadians(135));
        TrajectoryActionBuilder blueside6 = blueside5.endTrajectory().fresh()
                .splineTo(new Vector2d(-53, 40),Math.toRadians(225));
        TrajectoryActionBuilder blueside7 = blueside6.endTrajectory().fresh()
                .turnTo(Math.toRadians(135));
        TrajectoryActionBuilder blueside8 = blueside7.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-35, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(62)
                .waitSeconds(0.1)
                .lineToYConstantHeading(50)
                .strafeToSplineHeading(new Vector2d(-5, 40), Math.toRadians(90))
                .waitSeconds(0.1)
                .lineToYConstantHeading(33);
        TrajectoryActionBuilder blueside9 = blueside8.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-35, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(62)
                .waitSeconds(0.1)
                .lineToYConstantHeading(50)
                .strafeToSplineHeading(new Vector2d(-1, 40), Math.toRadians(90))
                .waitSeconds(0.1);
        TrajectoryActionBuilder blueside10 = blueside9.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-35, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(62)
                .waitSeconds(0.1)
                .lineToYConstantHeading(50)
                .strafeToSplineHeading(new Vector2d(3, 40), Math.toRadians(90))
                .waitSeconds(0.1);
        TrajectoryActionBuilder blueside11 = blueside10.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-35, 50), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToYConstantHeading(62)
                .waitSeconds(0.1)
                .lineToYConstantHeading(50)
                .strafeToSplineHeading(new Vector2d(7, 40), Math.toRadians(90))
                .waitSeconds(0.1);
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
        if (side == 0) {
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

        } else if (side == 1) {
            //redside.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        traj1,
                        LIFTFROGGY.liftup(),
                        LIFTFROGGY.liftback(),
                        traj2,
                        PUSHFROGGY.pushforward(),
                        traj3,
                        traj4,
                        traj5,
                        traj6,
                        traj7,
                        PUSHFROGGY.pushreturn(),
                        traj8,
                        LIFTFROGGY.liftup(),
                        LIFTFROGGY.liftback(),
                        traj9,
                        LIFTFROGGY.liftup(),
                        LIFTFROGGY.liftback(),
                        traj10,
                        LIFTFROGGY.liftup(),
                        LIFTFROGGY.liftback(),
                        traj11,
                        LIFTFROGGY.liftup(),
                        LIFTFROGGY.liftback()

                )
        );

    }

}