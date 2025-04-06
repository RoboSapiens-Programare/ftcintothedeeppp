package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {
    private boolean initialize;
    public Intake intake;
    public Outtake outtake;
    public UniversalTransfer universalTransfer;

    public Robot(HardwareMap hardwareMap){
        initialize = true;
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        universalTransfer = new UniversalTransfer(intake, outtake);
        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
