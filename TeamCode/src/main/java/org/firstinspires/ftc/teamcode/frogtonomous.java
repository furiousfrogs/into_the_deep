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
        public Action pushreturn(){
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
        Pose2d initialpose = new Pose2d(-10, 62, Math.toRadians(270));
        push PUSHFROGGY = new push(hardwareMap);

        MecanumDrive myBot = new MecanumDrive(hardwareMap, initialpose);

        TrajectoryActionBuilder blueside1 = myBot.actionBuilder(new Pose2d(-10, 62, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(-25, 60))
                .waitSeconds(1)
                .splineTo(new Vector2d(-48, 15), Math.toRadians(90))
                .waitSeconds(1);
        TrajectoryActionBuilder blueside2 = myBot.actionBuilder(new Pose2d(-48, 15, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-57, 15))
                .waitSeconds(1);
        TrajectoryActionBuilder blueside3 = myBot.actionBuilder(new Pose2d(-10, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(-35, 50), Math.toRadians(270))
                .lineToYConstantHeading(62);
//                    //just repeat 5 times but cannot use for loop :(
//                    .lineToYConstantHeading(50)
//                    .splineTo(new Vector2d(-6, 50), Math.toRadians(90))
//                    .lineToYConstantHeading(32);
        //.afterDisp(60, Action2);

        //move vertical slide

        TrajectoryActionBuilder bluesidefirst = myBot.actionBuilder(initialpose)
                .splineTo(new Vector2d(-35,40 ), Math.toRadians(225))
                .waitSeconds(0.5);
        TrajectoryActionBuilder bluesidefirstback = myBot.actionBuilder(initialpose)
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.5);
        TrajectoryActionBuilder bluesidesec = myBot.actionBuilder(initialpose)
                .lineToXSplineHeading(-40, Math.toRadians(225))
                .waitSeconds(0.5);
        TrajectoryActionBuilder bluesidesecback = myBot.actionBuilder(initialpose)
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.5);
        TrajectoryActionBuilder bluesidethird = myBot.actionBuilder(initialpose)
                .lineToXSplineHeading(-45, Math.toRadians(225))
                .waitSeconds(0.5);
        TrajectoryActionBuilder bluesidethirdback = myBot.actionBuilder(initialpose)
                .setTangent(Math.toRadians(135))
                .waitSeconds(0.5);


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

        Action traj1 = null;
        Action traj2 = null;
        Action traj3 = null;
        Action traj4 = null;
        Action traj5 = null;
        Action traj6 = null;
        if (side == 0) {
            traj1 = bluesidefirst.build();
            traj2 = bluesidefirstback.build();
            traj3 = bluesidesec.build();
            traj4 = bluesidesecback.build();
            traj5 = bluesidethird.build();
            traj6 = bluesidethirdback.build();
        } else if (side == 1) {
            redside.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        traj1,

                        traj2,
                        traj3,

                        traj4,

                        traj5,

                        traj6

                )
        );

    }

}