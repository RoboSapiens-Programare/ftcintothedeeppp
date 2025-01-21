package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class universalValues {

    public static double OUTTAKE_LOOSE = 0.865 ;
    public static double INTAKE_UP = 0.94;
    public static double INTAKE_DOWN = 0;
    public static double INTAKE_INT = 0.425;
    public static double INTAKE_INIT = 0.85;

    // PIVOT position for when the sample is to be transferred
    // TODO: check actual value for *_LOOSE and INTAKE_TRANSFER
    public static double INTAKE_TRANSFER = 0.79;


    public static double CLAW_LOOSE = 0.49;
    public static double CLAW_OPEN = 0.7;
    public static double CLAW_CLOSE = 0.4;

    public static double CLAW_VERTICAL = 0.4;
    public static double CLAW_HORIZONTAL = 0.72;
    public static double CLAW_FLIPPED = 0.08;

    public static double OUTTAKE_DUMP_BUCKET = 0.7;
    public static double OUTTAKE_COLLECT_NEW_TRANSFER = 0.39;
    public static double OUTTAKE_COLLECT = 0.04;
    public static double OUTTAKE_OPEN = 0.708;
    public static double OUTTAKE_OPEN_BAR = 0.60;
    public static double OUTTAKE_CLOSE = 0.91;

    // outtake values for clipping on specimen

    public static double OUTTAKE_CLIPON_UP = 0.65;
    public static double OUTTAKE_CLIPON_DOWN = 0.90;
    public static double OUTTAKE_PICKUP_BAR = 1;

    public static double CLAW_TIMER = 1.6;
    public static double SAMPLE_TIMER = 1.5;
    public static double PIVOT_TIMER = 0.0125;


    public static int INTAKE_EXTEND = 300;
    public static int OUTTAKE_EXTEND_MID = -1285;
    public static int OUTTAKE_EXTEND =  -1285;
    public static int OUTTAKE_RETRACT = -32;
    public static int INTAKE_RETRACT = 0;
}
