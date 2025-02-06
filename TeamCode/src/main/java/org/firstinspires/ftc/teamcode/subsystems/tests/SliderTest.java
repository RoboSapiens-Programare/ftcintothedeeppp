package org.firstinspires.ftc.teamcode.subsystems.tests;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Disabled
@Autonomous(name = "Slider Test", group = "Subsystem Tests")
public class SliderTest extends OpMode {
    private Robot robot;
    private Timer timer = new Timer();
    private int step = 0;

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
        if (step == 0) {
            timer.resetTimer();
            robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND, 0.8);
            ++step;
        }
        if (step == 1 && timer.getElapsedTimeSeconds() > 1) {
            robot.intake.ManualLevel(UniversalValues.INTAKE_RETRACT, 0.8);
            ++step;
        }

        if (step == 2 && timer.getElapsedTimeSeconds() > 2) {
            step = 0;
        }

    }
}
