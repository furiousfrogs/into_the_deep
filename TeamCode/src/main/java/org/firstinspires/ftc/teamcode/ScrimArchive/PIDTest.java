package org.firstinspires.ftc.teamcode.ScrimArchive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PIDTest", group="Linear Opmode")
public class PIDTest extends LinearOpMode {
    // Hardware
    private DcMotor viperSlideMotor;

    // PID Constants
    private static final double Kp = 0.01; // Proportional gain
    private static final double Ki = 0.0;  // Integral gain
    private static final double Kd = 0.001; // Derivative gain

    // PID Variables
    private double targetPosition = 0; // Target position in encoder ticks
    private double integral = 0;
    private double lastError = 0;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Get target position from gamepad buttons (adjust as necessary)
            if (gamepad1.dpad_up) {
                targetPosition += 100; // Move up
            } else if (gamepad1.dpad_down) {
                targetPosition -= 100; // Move down
            }

            // Clamp target position to avoid hardware damage
            targetPosition = Math.max(0, Math.min(targetPosition, 5000)); // Example limits

            // PID Control
            double currentPosition = viperSlideMotor.getCurrentPosition();
            double error = targetPosition - currentPosition;
            integral += error * runtime.seconds();
            double derivative = (error - lastError) / runtime.seconds();
            double power = Kp * error + Ki * integral + Kd * derivative;

            // Apply power to motor
            viperSlideMotor.setPower(power);

            // Update telemetry
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.update();

            // Reset timing and last error
            runtime.reset();
            lastError = error;
        }
    }
}