package pedroPathing.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class universalValues {

    public static double INTAKE_UP = 0.9;
    public static double INTAKE_DOWN = 0.06;
    public static double INTAKE_INT = 0.35;
    public static double INTAKE_INIT = 0.75;

    public static double CLAW_OPEN = 0.46;
    public static double CLAW_CLOSE = 0.545;
    public static double CLAW_VERTICAL = 0.4;
    public static double CLAW_HORIZONTAL = 0.72;

    public static double OUTTAKE_DUMP_BUCKET = 0.78;
    public static double OUTTAKE_COLLECT = 0.2;
    public static double OUTTAKE_OPEN = 0.725;
    public static double OUTTAKE_OPEN_BAR = 0.60;
    public static double OUTTAKE_CLOSE = 0.85;

    // outtake values for clipping on specimen

    public static double OUTTAKE_CLIPON_UP = 0.65;
    public static double OUTTAKE_CLIPON_DOWN = 0.90;
    public static double OUTTAKE_PICKUP_BAR = 1;

    public static double CLAW_TIMER = 1;
    public static double SAMPLE_TIMER = 1.5;
    public static double PIVOT_TIMER = 0.0125;


    public static int INTAKE_EXTEND = 1250;
    public static int OUTTAKE_EXTEND_MID = 800;
    public static int OUTTAKE_EXTEND = 2200;
}
