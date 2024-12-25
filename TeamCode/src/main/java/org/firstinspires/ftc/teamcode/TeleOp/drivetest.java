//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//
//@TeleOp(name = "drivetest", group = "TeleOp")
//public class drivetest extends OpMode {
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//
//    @Override
//    public void init() {
//        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontRight = hardwareMap.get(DcMotor.class, "front_right");
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        backLeft = hardwareMap.get(DcMotor.class, "back_left");
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        backRight = hardwareMap.get(DcMotor.class, "back_right");
//        backRight.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        public void MecanumDrive(double vertical, double horizontal, double clockwise, double maxSpeed) {
//            frontRight.setPower((-clockwise + horizontal + vertical) * maxSpeed);
//            backRight.setPower((-clockwise - horizontal + vertical) * maxSpeed);
//            frontLeft.setPower(-(-clockwise + horizontal - vertical) * maxSpeed);
//            backLeft.setPower(-(-clockwise - horizontal - vertical) * maxSpeed);
//        }
//
//    @Override
//    public void loop() {
//        init();
//    }
//}
