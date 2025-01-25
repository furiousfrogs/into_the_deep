package org.firstinspires.ftc.teamcode.codeMethodized;

public class Calculate {
    private Drive drivModule;
    private Take takeModule;
    private Telemetry teleModule;
    public void Actions(Initialization initModule){
        drivModule.Drive(initModule);
        takeModule.manualTake(initModule);
        teleModule.telemetry(initModule);
        initModule.previousGamepad1.copy(initModule.currentGamepad1);
    }
}
