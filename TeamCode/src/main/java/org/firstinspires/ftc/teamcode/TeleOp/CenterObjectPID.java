/*
README
I put this here cuz our shit is unorganizes as hell like wtf are all these random files. just put this where you want it i think
A lot of stuff needs to be tuned so example the p,i,d values, target x if its not the center, base power, and gamepad buttons changed to perferance
Right now it should only work for one object so test with one object
 */


package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
public class CenterObjectPID extends OpMode {

    private HuskyLens huskyLens;

    //tune this or whatever
    private double Kp = 0.005;
    private double Ki = 0.0001;
    private double Kd = 0.01;

    private double targetX = 160; //center of the screen (length screen/2) adjust accordingly
    private double basePower = 0.3; //adjust accordingly

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void loop() {
        if (gamepad1.a) { //change this to whatever button you want

            HuskyLens.Block[] blocks = huskyLens.getRecognizedBlocks();
            HuskyLens.Block yellowBlock = blocks[0]; //getting the first element for now

            double currentX = yellowBlock.x;
            double error = targetX - currentX;

            integral += error;
            double derivative = error - previousError;

            //PID = P+i+d
            double turnPower = (Kp * error) + (Ki * integral) + (Kd * derivative);

            turnPower = Math.max(-basePower, Math.min(basePower, turnPower));

            //right now its a +- of ten degrees so change accordingly
            if (error > 10) {
                rotateRight(turnPower);
            } else if (error < -10) {
                rotateLeft(turnPower);
            } else {
                stopMotors();
            }

            telemetry.addData("Current X", currentX);
            telemetry.addData("Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
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
