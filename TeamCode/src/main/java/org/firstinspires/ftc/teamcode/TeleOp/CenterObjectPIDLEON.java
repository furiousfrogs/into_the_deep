/*
README
I put this here cuz our shit is unorganizes as hell like wtf are all these random files. just put this where you want it i think
A lot of stuff needs to be tuned so example the p,i,d values, target x if its not the center, base power, and gamepad buttons changed to perferance
Right now it should only work for one object so test with one object
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "CenterObjectPID", group = "TeleOp")
public class CenterObjectPIDLEON extends OpMode {

    private static final int READ_PERIOD = 2; // Rate limit in seconds
    private Deadline rateLimit;

    private HuskyLens huskyLens;

    // Tune these values
    private double Kp = 0.005;
    private double Ki = 0.0001;
    private double Kd = 0.01;

    private double targetX = 160; // Center of the screen (length screen/2) adjust accordingly
    private double basePower = 0.3; // Adjust accordingly

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    HuskyLens.Block yellowBlock;

    private double integral = 0, previousError = 0;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Initialize the rate limiter
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
    }

    @Override
    public void loop() {
        if (!rateLimit.hasExpired()) {
            return; // Exit loop early if rate limit hasn't expired
        }
        rateLimit.reset(); // Reset the rate limit timer

        if (gamepad1.a) { // Change this to your preferred button

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length == 0) {
                telemetry.addLine("Nothing Detected");
                telemetry.update();
                return;
            }




            yellowBlock = blocks[0]; // Process the first detected block
            double currentX = yellowBlock.x;
            double error = targetX - currentX;

            // PID calculations
            integral += error;
            double derivative = error - previousError;
            double turnPower = (Kp * error) + (Ki * integral) + (Kd * derivative);
            turnPower = Math.max(-basePower, Math.min(basePower, turnPower));

            if (error > 10) {
                telemetry.addLine("Block Detected, Rotating Right");
                rotateRight(turnPower);
            } else if (error < -10) {
                telemetry.addLine("Block Detected, Rotating Left");
                rotateLeft(turnPower);
            } else {
                telemetry.addLine("Centered, Object Centered");
                stopMotors();
            }

            telemetry.addData("Current X", currentX);
            telemetry.addData("Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();

            previousError = error; // Update previous error for derivative calculation
        }
    }

    private void rotateRight(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }

    private void rotateLeft(double power) {
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
