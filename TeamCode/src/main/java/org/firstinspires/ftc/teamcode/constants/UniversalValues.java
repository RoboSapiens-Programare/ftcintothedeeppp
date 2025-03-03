package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class UniversalValues {

    public static double INTAKE_UP = 0.73;
    public static double INTAKE_DOWN = 0.14;
    public static double INTAKE_INT = 0.5;
    public static double INTAKE_INIT = 0.937;

    // PIVOT position for when the sample is to be transferred
    // TODO: check actual value for *_LOOSE and INTAKE_TRANSFER
    public static double INTAKE_TRANSFER = 0.937;

    public static double CLAW_CLOSE = 0.1;
    public static double CLAW_LOOSE = 0.212;
    public static double CLAW_OPEN = 0.45;

    public static double CLAW_VERTICAL = 0.4;
    public static double CLAW_HORIZONTAL = 0.72;
    public static double CLAW_FLIPPED = 0.08;

    public static double OUTTAKE_DUMP_BUCKET = 0.71;
    public static double OUTTAKE_DUMP_BUCKET_DIAG = 0.78;
    public static double OUTTAKE_COLLECT_NEW_TRANSFER = 0.37;
    public static double OUTTAKE_COLLECT = 0.04;

    // TODO: review actual position
    public static double OUTTAKE_GRAB_BAR = 1;

    public static double OUTTAKE_OPEN = 0.3;
    public static double OUTTAKE_OPEN_BAR = 0.60;
    public static double OUTTAKE_CLOSE = 0.47;

    // outtake values for clipping on specimen

    public static double OUTTAKE_CLIPON_UP = 0.65;
    public static double OUTTAKE_CLIPON_DOWN = 0.90;
    public static double OUTTAKE_PICKUP_BAR = 1;

    public static double CLAW_TIMER = 1.6;
    public static double SAMPLE_TIMER = 1.5;
    public static double PIVOT_TIMER = 0.0125;


    public static int INTAKE_EXTEND = 290;

    public static int OUTTAKE_EXTEND_MID = -1770;
    public static int OUTTAKE_EXTEND_GRAB = -1584;
    public static int OUTTAKE_ASCENT = -754;
    public static int OUTTAKE_EXTEND_SPECIMEN = -600;
    public static int OUTTAKE_EXTEND =  -1770;
    public static int OUTTAKE_EXTEND_ASCENT_INTERMEDIARY = -1000;

    public static int OUTTAKE_RETRACT = 0;

    public static int INTAKE_RETRACT = 0;
}
