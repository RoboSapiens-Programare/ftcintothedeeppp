package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class robot {
    private boolean initialize;
    public org.firstinspires.ftc.teamcode.subsystems.intake intake;
    public org.firstinspires.ftc.teamcode.subsystems.outtake outtake;

    public robot(HardwareMap hardwareMap){
        initialize = true;
        intake = new intake(hardwareMap);
        outtake = new outtake(hardwareMap);
        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
