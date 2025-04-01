package org.firstinspires.ftc.teamcode.subsystems.tests;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

// @Disabled
@Autonomous(name = "Transfer Emulator Test", group = "Subsystem Tests")
public class TransferEmulatorTest extends OpMode {
    private Robot robot;
    private Timer timer = new Timer();
    private int transferStep = 0;
    private boolean timerSingleton = true;
    private boolean transferCompleted = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
        robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND,0.75);
        robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
        robot.intake.setPivot(UniversalValues.INTAKE_INT);
        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
    }

    @Override
    public void loop() {
        if (timerSingleton) {
            timerSingleton = false;
            timer.resetTimer();
        }

        if (transferStep == 0) {
            robot.intake.setClawPivot(UniversalValues.CLAW_VERTICAL + 0.2);

            robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
            robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
            robot.intake.OpenIntake(UniversalValues.CLAW_LOOSE);

            ++transferStep;
        }
        if (timer.getElapsedTimeSeconds() > 0.4 && transferStep == 1) {
            robot.intake.ManualLevel(UniversalValues.INTAKE_RETRACT, 0.6);

            robot.intake.setPivot(1);
            robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 0.9 && transferStep ==  2) {
            robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 1.3 && transferStep == 3) {
            robot.intake.setPivot(UniversalValues.INTAKE_TRANSFER - 0.2);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 1.5 && transferStep == 4) {
            robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 1.9 && transferStep == 5) {
            robot.intake.setPivot(UniversalValues.INTAKE_TRANSFER);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 2.1 && transferStep == 6) {
            robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 2.5 && transferStep == 7) {
            robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 2.7 && transferStep == 8) {
            robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
            robot.intake.setPivot(UniversalValues.INTAKE_INT);

            transferStep = -1;
            transferCompleted = true;
        }

    }
}
