package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {
    private boolean initialize;
    public Intake intake;
    public Outtake outtake;
    public TwistTransfer universalTransfer;

    public Robot(HardwareMap hardwareMap){
        initialize = true;
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        universalTransfer = new TwistTransfer(intake, outtake);
        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
