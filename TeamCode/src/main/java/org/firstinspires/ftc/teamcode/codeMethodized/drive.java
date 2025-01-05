package org.firstinspires.ftc.teamcode.codeMethodized;

public class drive {

    public void driver(Initialization initModule) {
        double turnvar = Math.max(1, 1 + (initModule.horSlide.getCurrentPosition() / 1000.0));

        boolean slow = initModule.gamepad1.options;
        double y = -initModule.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = initModule.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx =  (initModule.gamepad1.right_trigger - initModule.gamepad1.left_trigger) / turnvar;


        double slowvar = 2.0; // Slow mode divisor
        double speedFactor = slow ? 1 / slowvar : 1;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.8);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double frontLeftPower = (y + x + rx) / denominator * speedFactor * 0.8;
        double backLeftPower = (y - x + rx) / denominator * speedFactor * 0.8;
        double frontRightPower = (y - x - rx) / denominator * speedFactor * 0.8;
        double backRightPower = (y + x - rx) / denominator * speedFactor * 0.8;


        // Normalize motor powers
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        if (initModule.vertSlideL.getCurrentPosition() > 2000) {
            frontLeftPower = frontLeftPower/2;
            frontRightPower = frontRightPower/2;
            backLeftPower = backLeftPower/2;
            backRightPower = backRightPower/2;
        }
        // Set motor powers
        initModule.frontLeft.setPower(frontLeftPower);
        initModule.frontRight.setPower(frontRightPower);
        initModule.backLeft.setPower(backLeftPower);
        initModule.backRight.setPower(backRightPower);
    }
}
