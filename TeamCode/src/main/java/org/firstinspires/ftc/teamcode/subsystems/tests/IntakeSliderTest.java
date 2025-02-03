package org.firstinspires.ftc.teamcode.subsystems.tests;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.robot;

@Disabled
@Autonomous(name = "Intake Slider Test", group = "Subsystem Tests")
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
        robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(UniversalValues.INTAKE_INT);
        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);

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

            robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND, 0.75);
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