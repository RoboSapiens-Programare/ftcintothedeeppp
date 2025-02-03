package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

@Autonomous(name = "Claw Test", group = "Subsystem Tests")
public class ClawTest extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot;

    private int step = 0;
    private final Timer timer = new Timer();

    @Override
    public void init() {
        robot = new robot(hardwareMap);

        robot.intake.ManualLevel(universalValues.INTAKE_RETRACT,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(universalValues.INTAKE_INT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
    }

    @Override
    public void loop() {

        if (step == 0) {
            robot.intake.OpenIntake(universalValues.CLAW_OPEN);
            ++step;
        }
        if (timer.getElapsedTimeSeconds() > 1 && step == 1) {
            robot.intake.OpenIntake(universalValues.CLAW_LOOSE);
            ++step;
        }
        if (timer.getElapsedTimeSeconds() > 2 && step == 2) {
            robot.intake.OpenIntake(universalValues.CLAW_CLOSE);
            ++step;
        }
        if (timer.getElapsedTimeSeconds() > 3 && step == 3) {
            timer.resetTimer();
            step = 0;
        }


//        robot.intake.OpenIntake(universalValues.CLAW_OPEN);
    }
}
