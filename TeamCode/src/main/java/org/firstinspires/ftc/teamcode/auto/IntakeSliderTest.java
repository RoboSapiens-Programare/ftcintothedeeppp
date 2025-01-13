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
    private boolean singleton, singleton2;

    // will wait {duration} minutes
    private final double duration = 30;

    @Override
    public void init() {
        robot = new robot(hardwareMap);

        timeoutTimer = new Timer();
        actionTimer = new Timer();

        timeoutTimer.resetTimer();
        actionTimer.resetTimer();

        singleton = true;
        singleton2 = true;

        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
            robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
            robot.intake.setPivot(universalValues.INTAKE_INT);
            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
            robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

        telemetry.update();
    }

    @Override
    public void loop() {
        if (timeoutTimer.getElapsedTimeSeconds() >= duration*60) {
            return;
        }
        if (singleton) {

            robot.intake.ManualLevel(universalValues.INTAKE_EXTEND, 1);
            singleton = false;
        }
        if (actionTimer.getElapsedTimeSeconds() > 3) {
            if (singleton2) {
                robot.intake.ManualLevel(0, 1);
                singleton2 = false;
            }
            if (actionTimer.getElapsedTimeSeconds() > 6) {
                actionTimer.resetTimer();
                singleton = true;
                singleton2 = true;
            }
        }
        int mins = timeoutTimer.getElapsedTimeSeconds()/60;
        int secs = (int)timeoutTimer.getElapsedTimeSeconds() % 60;
        String time = Integer.toString(mins)+":"+Integer.toString(secs);
        telemetry.addData("elapsed time", time);
        telemetry.update();
    }
}
