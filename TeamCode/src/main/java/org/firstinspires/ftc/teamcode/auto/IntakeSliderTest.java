package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Intake Slider Test", group = "Autonomous")
public class IntakeSliderTest extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot = null;

    private Timer timeoutTimer, actionTimer;
    private boolean singletonExtend, singletonRetract;

    // will wait {duration} minutes
    private final double  duration = 30;

    @Override
    public void init() {
        robot = new robot(hardwareMap);

        timeoutTimer = new Timer();
        actionTimer = new Timer();

        timeoutTimer.resetTimer();
        actionTimer.resetTimer();

        singletonExtend = true;
        singletonRetract = true;

        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(universalValues.INTAKE_INT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

        DcMotorEx intakeMotor;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();
    }

    @Override
    public void loop() {
        if (timeoutTimer.getElapsedTimeSeconds() >= duration*60) {
            return;
        }
        if (singletonExtend) {

            robot.intake.ManualLevel(universalValues.INTAKE_EXTEND, 0.75);
            singletonExtend = false;
        }
        if (actionTimer.getElapsedTimeSeconds() > 3) {
            if (singletonRetract) {
                robot.intake.ManualLevel(0, 0.75);
                singletonRetract = false;
            }
            if (actionTimer.getElapsedTimeSeconds() > 6) {
                actionTimer.resetTimer();
                singletonExtend = true;
                singletonRetract = true;
            }
        }
        int mins = (int) (timeoutTimer.getElapsedTimeSeconds()/60);
        int secs = (int)timeoutTimer.getElapsedTimeSeconds() % 60;
        String time = mins +":"+ secs; // no need for Integer.toString apparently
        telemetry.addData("elapsed time", time);
        telemetry.update();
    }
}