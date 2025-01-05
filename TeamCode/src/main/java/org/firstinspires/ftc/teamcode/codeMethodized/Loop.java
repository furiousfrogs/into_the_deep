package org.firstinspires.ftc.teamcode.codeMethodized;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FFVar;

public class Loop {

    public void handleLoop(Initialization initModule) {
        int red = initModule.coloursensor.red();
        int green = initModule.coloursensor.green();
        int blue = initModule.coloursensor.blue();
        // Store previous state
        initModule.previousGamepad1.copy(initModule.currentGamepad1);
        // Update current state with the latest gamepad data
        initModule.currentGamepad1.copy(initModule.gamepad1);

        // Store previous state
        initModule.previousGamepad2.copy(initModule.currentGamepad2);
        // Update current state with the latest gamepad data
        initModule.currentGamepad2.copy(initModule.gamepad2);

        if (initModule.gamepad2.options && !initModule.intaking && !initModule.outtaking) {
            initModule.outArm.setPosition(FFVar.ArmOut);
            initModule.wrist.setPosition(FFVar.WristOut);
            initModule.claw.setPosition(FFVar.ClawOpen);
        }

        if (initModule.currentGamepad2.right_bumper && !initModule.previousGamepad2.right_bumper) { //Intake
            if (initModule.intake.getPower() < 0.2) {
                initModule.intake.setPower(0.8);
                initModule.intaking = false;
            } else {
                initModule.intake.setPower(0);
                initModule.intaking = false;
            }
        }


        if (initModule.currentGamepad2.left_bumper && !initModule.previousGamepad2.left_bumper) {
            if (initModule.intake.getPower() > -0.2) {
                initModule.intake.setPower(-0.8); // Reverse
                initModule.intaking = true;
            } else {
                initModule.intake.setPower(0); // Stop
                initModule.intaking = false;
            }
        }

        if (initModule.intaking && red > green && red > blue && red > 500) { //detects red
            initModule.intake.setPower(0);
            initModule.intaking = false;
        }
        if (initModule.intaking && blue > red && blue > green && blue > 500) { //detects blue
            initModule.intake.setPower(0);
            initModule.intaking = false;
        }


        if (initModule.intaking && red > 120 && green > 120 && blue < 120) { //detects yellow
            initModule.intake.setPower(0);
            initModule.intaking = false;
        }


        if (initModule.currentGamepad2.cross && !initModule.previousGamepad2.cross && !initModule.transfering) { // Arm down
            if (initModule.leftIn.getPosition() > 0.5) {
                // Set servo positions to ArmDwn
                initModule.leftIn.setPosition(FFVar.InWait);
                initModule.rightIn.setPosition(FFVar.InWait);
            } else if ((initModule.leftIn.getPosition() < 0.5) && Math.abs(initModule.horSlide.getCurrentPosition()) > 350) {
                // Set servo positions to ArmUp
                initModule.leftIn.setPosition(FFVar.InDown);
                initModule.rightIn.setPosition(FFVar.InDown);
            }
        }


        if (initModule.hortouch.isPressed()) { //Horizontal touch sensor detection
            initModule.horSlide.setPower(0);
            initModule.horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.horSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (initModule.vertouch.isPressed()) { //Reset vertical encoders
            initModule.vertSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.vertSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            initModule.vertSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            initModule.vertSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (initModule.horSlide.getCurrentPosition() < 550 && initModule.leftIn.getPosition() > FFVar.InWait) {
            initModule.leftIn.setPosition(FFVar.InWait);
            initModule.rightIn.setPosition(FFVar.InWait);
        }

        if (initModule.currentGamepad2.circle && !initModule.previousGamepad2.circle && initModule.sample) {
            initModule.outtaking = true;
            // Start the Outtake action
            initModule.outArm.setPosition(FFVar.ArmOut2);
            initModule.wrist.setPosition(FFVar.WristOut);
            initModule.claw.setPosition(FFVar.ClawClose);
            initModule.OuttakeTimer.reset(); // Reset the timer
            initModule.OuttakeAction = true; // Mark the action as in progress
        }

// Check if Outtake action is in progress
        if (initModule.OuttakeAction) {
            if (initModule.OuttakeTimer.seconds() >= FFVar.OuttakeTime) {
                // Perform the next step of the action
                initModule.claw.setPosition(FFVar.ClawOpen);
                initModule.OuttakeAction = false; // End the action
                initModule.OuttakeAction2 = true;
                initModule.OuttakeTimer2.reset();
            }
        }

        if (initModule.OuttakeAction2) {
            if (initModule.OuttakeTimer2.seconds() >= FFVar.OuttakeTime2) {
                initModule.claw.setPosition(FFVar.ClawClose);
                initModule.outArm.setPosition(FFVar.ArmWait);
                initModule.wrist.setPosition(FFVar.WristOut);

                initModule.OuttakeAction2 = false;
                initModule.sample = false;
                initModule.outtaking = false;
            }
        }

        if ((initModule.currentGamepad2.triangle && !initModule.previousGamepad2.triangle) || (initModule.currentGamepad1.triangle && !initModule.previousGamepad1.triangle)) {
            if (!initModule.hortouch.isPressed()) {
                initModule.horSlide.setPower(-1);
            }
            if (!initModule.vertouch.isPressed()) {
                initModule.vertSlideR.setPower(-1);
                initModule.vertSlideL.setPower(-1);
            }
            initModule.resethor = true;
            initModule.resetver = true;
        }

        if (((initModule.currentGamepad2.square && !initModule.previousGamepad2.square) || (initModule.currentGamepad1.square && !initModule.previousGamepad1.square))  && !initModule.transfering && !initModule.outtaking) {
            initModule.transfering = true;
            if (initModule.horSlide.getCurrentPosition() >= 350) {
                initModule.leftIn.setPosition(FFVar.InWait);
                initModule.rightIn.setPosition(FFVar.InWait);
            } else {
                initModule.leftIn.setPosition(FFVar.InWait2);
                initModule.rightIn.setPosition(FFVar.InWait2);
            }
            if (!initModule.hortouch.isPressed()) {
                initModule.horSlide.setPower(-1);
            }
            if (!initModule.vertouch.isPressed()) {
                initModule.vertSlideR.setPower(-1);
                initModule.vertSlideL.setPower(-1);
            }
            // Move the arm and wrist to their positions
            initModule.outArm.setPosition(FFVar.ArmTransfer);
            initModule.wrist.setPosition(FFVar.WristTransfer);
            initModule.claw.setPosition(FFVar.ClawOpen);
            initModule.intake.setPower(0);


            // Initialize reset flags
            initModule.resethor = true;
            initModule.resetver = true;


            initModule.Transfer1action = true; // Start a new action sequence
            initModule.Transfer1Timer.reset();
        }

// Check if an action is in progress
        if (initModule.Transfer1action) {
            if (initModule.hortouch.isPressed() && initModule.vertouch.isPressed() && initModule.Transfer1Timer.seconds() > FFVar.TransferATime) {

                initModule.leftIn.setPosition(FFVar.InTransfer);
                initModule.rightIn.setPosition(FFVar.InTransfer);

                // Set the positions for leftIn and rightIn after delay


                initModule.Transfer2action = true;
                initModule.Transfer2Timer.reset();

                // End the action
                initModule.Transfer1action = false;
            }
        }
        if (initModule.Transfer2action) {

            if (initModule.Transfer2Timer.seconds() >= FFVar.TransferBTime) {
                initModule.claw.setPosition(FFVar.ClawClose);


                initModule.Transfer3action = true;
                initModule.Transfer3Timer.reset();

                // End the action
                initModule.Transfer2action = false;
            }
        }
        if (initModule.Transfer3action) {
            if (initModule.Transfer3Timer.seconds() >= FFVar.TransferCTime) {

                initModule.outArm.setPosition(FFVar.ArmWait);
                initModule.wrist.setPosition(FFVar.WristTransfer2);
                initModule.leftIn.setPosition(FFVar.InWait);
                initModule.rightIn.setPosition(FFVar.InWait);
                initModule.Transfer3action = false;

                initModule.Transfer4Timer.reset();
                initModule.Transfer4action = true;
            }

        }

        if (initModule.Transfer4action) {
            if (initModule.Transfer4Timer.seconds() >= FFVar.TransferDTime) {

                initModule.wrist.setPosition(FFVar.WristOut);
                initModule.outArm.setPosition(FFVar.ArmOut);
                initModule.leftIn.setPosition(FFVar.InUp);
                initModule.rightIn.setPosition(FFVar.InUp);
                initModule.Transfer4action = false;

                initModule.transfering = false;
                initModule.sample = true;
            }
        }
// Handle horizontal slide reset logic
        if (initModule.hortouch.isPressed()) {
            initModule.resethor = false;
        }
        if (initModule.vertouch.isPressed()) {
            initModule.resetver = false;
        }
        if (!initModule.resethor && !initModule.transfering) {
            if (!initModule.limitCalculated) {
                int speedBuffer = (int) (initModule.horSlide.getPower() * 230); // Buffer proportional to speed (tune the factor)
                initModule.dynamicLimit = 1200 - speedBuffer;
                initModule.limitCalculated = true; // Lock the limit while the slide is moving
            }
            if (initModule.horSlide.getCurrentPosition() > initModule.dynamicLimit) {
                if (-initModule.gamepad2.left_stick_x < 0) {
                    initModule.horSlide.setPower(-initModule.gamepad2.left_stick_x);
                } else {
                    initModule.horSlide.setPower(0);
                }
            } else {
                initModule.horSlide.setPower(-initModule.gamepad2.left_stick_x); // Horizontal slide
                if (-initModule.gamepad2.left_stick_x != 0 && !initModule.hortouch.isPressed() && initModule.leftIn.getPosition() < 0.3) {
                    initModule.leftIn.setPosition(FFVar.InWait);
                    initModule.rightIn.setPosition(FFVar.InWait);
                } else if (initModule.hortouch.isPressed()) {
                    initModule.leftIn.setPosition(FFVar.InUp);
                    initModule.rightIn.setPosition(FFVar.InUp);
                }
            }

        }
        if (Math.abs(-initModule.gamepad2.left_stick_x) < 0.1) {
            initModule.limitCalculated = false; // Allow recalculation of the limit
        }
// Handle vertical slide reset logic

        if (!initModule.resetver && !initModule.transfering) {
            if (initModule.vertSlideR.getCurrentPosition() > 4000) {
                if (initModule.gamepad2.left_stick_y > 0) {
                    initModule.vertSlideL.setPower(-initModule.gamepad2.left_stick_y); // Vertical slide
                    initModule.vertSlideR.setPower(-initModule.gamepad2.left_stick_y);
                } else {
                    initModule.vertSlideL.setPower(0); // Vertical slide
                    initModule.vertSlideR.setPower(0);
                }
            } else {
                initModule.vertSlideL.setPower(-initModule.gamepad2.left_stick_y); // Vertical slide
                initModule.vertSlideR.setPower(-initModule.gamepad2.left_stick_y);
            }

        }



        if (initModule.gamepad1.dpad_up) {
            FFVar.targetPosition += 100; // Move up by 100 encoder ticks
        } else if (initModule.gamepad1.dpad_down) {
            FFVar.targetPosition -= 100; // Move down by 100 encoder ticks
        }

// Clamp target position to safe limits
        FFVar.targetPosition = Math.max(0, Math.min(4000, FFVar.targetPosition)); // Adjust range as needed
    }
}