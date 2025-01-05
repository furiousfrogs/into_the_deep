package org.firstinspires.ftc.teamcode.codeMethodized;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.FFVar;

public class telemetrics {

    public void telem(Initialization initModule) {
        telemetry.addData("Vertical Right Slide Pos", initModule.vertSlideR.getCurrentPosition());
        telemetry.addData("Vertical left Slide Pos", initModule.vertSlideL.getCurrentPosition());
        telemetry.addData("Horizontal Slide Pos", initModule.horSlide.getCurrentPosition());
        telemetry.addData("hor power", initModule.horSlide.getPower());
        telemetry.addData("ver left power", initModule.vertSlideL.getPower());
        telemetry.addData("ver right power", initModule.vertSlideR.getPower());

        if (initModule.intaking) {
            telemetry.addLine("intaking");
        }
        if (initModule.hortouch.isPressed()) {
            telemetry.addLine("touch joe");
        }
        if (initModule.vertouch.isPressed()) {
            telemetry.addLine("touch chuck");
        }
        if (initModule.leftIn.getPosition() == FFVar.InTransfer) {
            telemetry.addLine("Intake Up");
        } else if (initModule.leftIn.getPosition() == FFVar.InDown){
            telemetry.addLine("Intake down");
        }
    }
}
