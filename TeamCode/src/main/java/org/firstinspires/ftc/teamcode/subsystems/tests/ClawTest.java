package org.firstinspires.ftc.teamcode.subsystems.tests;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

//@Disabled
@Autonomous(name = "Claw Test", group = "Subsystem Tests")
public class ClawTest extends OpMode {
    private Robot robot;

    private int step = 0;

    private final Timer timer = new Timer();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.intake.ManualLevel(UniversalValues.INTAKE_RETRACT,1);
        robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(UniversalValues.INTAKE_INT);
        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
    }

    @Override
    public void loop() {
        /*
        if (step == 0) {
            robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
            ++step;
        }
        if (timer.getElapsedTimeSeconds() > 1 && step == 1) {
            robot.intake.OpenIntake(UniversalValues.CLAW_LOOSE);
            ++step;
        }
        if (timer.getElapsedTimeSeconds() > 2 && step == 2) {
            robot.intake.OpenIntake(UniversalValues.CLAW_CLOSE);
            ++step;
        }
        if (timer.getElapsedTimeSeconds() > 3 && step == 3) {
            timer.resetTimer();
            step = 0;
        }
        */

        robot.intake.OpenIntake(UniversalValues.CLAW_CLOSE);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
    }
}
