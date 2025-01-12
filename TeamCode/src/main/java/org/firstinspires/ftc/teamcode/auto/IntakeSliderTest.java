package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

import com.pedropathing.util.Timer;

@Autonomous(name = "Intake Slider Test", group = "Autonomous")
public class IntakeSliderTest extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot = null;

    private Timer timeoutTimer, actionTimer;

    // will wait {duration} minutes
    private final double duration = 30;

    @Override
    public void init() {
        robot = new robot(hardwareMap);

        timeoutTimer = new Timer();
        actionTimer = new Timer();

        timeoutTimer.resetTimer();
        actionTimer.resetTimer();

        robot.intake.ManualLevel(0,1);

        telemetry.update();
    }

    @Override
    public void loop() {
        if (timeoutTimer.getElapsedTimeSeconds() >= duration*60) {
            return;
        }

        robot.intake.ManualLevel(universalValues.INTAKE_EXTEND, 1);
        if (actionTimer.getElapsedTimeSeconds() > 3) {
            robot.intake.ManualLevel(0, 1);
            if (actionTimer.getElapsedTimeSeconds() > 6) {
                actionTimer.resetTimer();
            }
        }

        telemetry.addData("elapsed seconds", timeoutTimer.getElapsedTimeSeconds());
        telemetry.addData("elapsed minutes", timeoutTimer.getElapsedTimeSeconds()/60);
        telemetry.update();
    }
}
