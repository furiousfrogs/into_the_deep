package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp(name = "FTCLib Mecanum Drive", group = "TeleOp")
public class FTCLibMecanumDrive extends OpMode {

    // Motors
    private MotorEx frontLeft, frontRight, backLeft, backRight;

    // Mecanum Drive Object
    private MecanumDrive mecanumDrive;

    @Override
    public void init() {
        // Initialize motors
        frontLeft = new MotorEx(hardwareMap, "front_left");
        frontRight = new MotorEx(hardwareMap, "front_right");
        backLeft = new MotorEx(hardwareMap, "back_left");
        backRight = new MotorEx(hardwareMap, "back_right");



        // Initialize the mecanum drive
        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        telemetry.addLine("Initialized FTCLib Mecanum Drive");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Use gamepad inputs for movement
        double y = gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x * 1.1; // Strafing (adjust for imperfect strafing)
        double turn = gamepad1.left_trigger - gamepad1.right_trigger; // Rotation

        // Drive the robot
        mecanumDrive.driveRobotCentric(x, y, turn);

        // Telemetry for debugging
        telemetry.addData("y", y);
        telemetry.addData("x", x);
        telemetry.addData("turn", turn);
        telemetry.update();
    }
}