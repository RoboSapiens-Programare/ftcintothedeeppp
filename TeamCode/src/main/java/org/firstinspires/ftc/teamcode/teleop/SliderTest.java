package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

@Autonomous(name = "Slider Test", group = "Subsystem Tests")
public class SliderTest extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot;
    private Timer timer = new Timer();
    private int step = 0;

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
            timer.resetTimer();
            robot.intake.ManualLevel(universalValues.INTAKE_EXTEND, 0.8);
            ++step;
        }
        if (step == 1 && timer.getElapsedTimeSeconds() > 1) {
            robot.intake.ManualLevel(universalValues.INTAKE_RETRACT, 0.8);
            ++step;
        }

        if (step == 2 && timer.getElapsedTimeSeconds() > 2) {
            step = 0;
        }

    }
}
