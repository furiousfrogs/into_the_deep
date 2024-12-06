package org.firstinspires.ftc.teamcode.ScrimArchive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled
@Autonomous
public class FrogAuto extends LinearOpMode {

        private DcMotor frontLeft, frontRight, backLeft, backRight;
        private DcMotor vertSlideL, vertSlideR;

        @Override
        public void runOpMode() {

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

                vertSlideL = hardwareMap.get(DcMotor.class, "leftvertical");
                vertSlideL.setDirection(DcMotor.Direction.REVERSE);
                vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertSlideL.setTargetPosition(0);
                vertSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                vertSlideR = hardwareMap.get(DcMotor.class, "rightvertical");
                vertSlideR.setDirection(DcMotor.Direction.FORWARD);
                vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vertSlideR.setTargetPosition(0);
                vertSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                waitForStart();
                if (opModeIsActive()) {
                        // Create target positions

                        frontLeft.setPower(-0.5);//giving output
                        frontRight.setPower(-0.5);
                        backLeft.setPower(-0.5);
                        backRight.setPower(-0.5);
                        sleep(900);
                        vertSlideL.setPower(0.8);
                        vertSlideR.setPower(0.8);
                        vertSlideL.setTargetPosition(1350);
                        vertSlideR.setTargetPosition(1350);
                        frontLeft.setPower(0);//giving output
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);


                        sleep(2000);

                        frontLeft.setPower(-0.5);//giving output
                        frontRight.setPower(-0.5);
                        backLeft.setPower(-0.5);
                        backRight.setPower(-0.5);
                        sleep(280);
                        frontLeft.setPower(0);//giving output
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        vertSlideL.setPower(0.5);
                        vertSlideR.setPower(0.5);
                        vertSlideL.setTargetPosition(800);
                        vertSlideR.setTargetPosition(800);
                        sleep(2000);
                        frontLeft.setPower(0.5);//giving output
                        frontRight.setPower(0.5);
                        backLeft.setPower(0.5);
                        backRight.setPower(0.5);
                        sleep(1000);
                        //finsihed the hang

                        frontLeft.setPower(-0.5);
                        frontRight.setPower(0.5);
                        backLeft.setPower(-0.5);
                        backRight.setPower(0.5);
                        sleep(500);

                        frontLeft.setPower(0.5);
                        frontRight.setPower(0.5);
                        backLeft.setPower(0.5);
                        backRight.setPower(0.5);

                        sleep(2000);

                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);


                }
        }
}