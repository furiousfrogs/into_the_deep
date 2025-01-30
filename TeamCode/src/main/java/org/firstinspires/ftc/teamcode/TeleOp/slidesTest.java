package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//importing libraries
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "slidesTest",group = "TeleOp")
public class slidesTest extends OpMode {
    private DcMotor MotorDc;
    @Override
    public void init() {
        MotorDc=hardwareMap.get(DcMotor.class,"MotorDc");
        MotorDc.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        MotorDc.setPower(gamepad1.left_stick_x);
    }
}
