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

            robot.intake.OpenIntake(UniversalValues.CLAW_LOOSE);
            robot.intake.setClawPivot(UniversalValues.CLAW_VERTICAL + 0.2);

            robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
            robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);

            ++transferStep;
        }
        if (timer.getElapsedTimeSeconds() > 0.4) {
            robot.intake.ManualLevel(UniversalValues.INTAKE_RETRACT, 0.8);

            if (transferStep == 1) {
                robot.intake.setPivot(UniversalValues.INTAKE_TRANSFER);
                robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);

                ++transferStep;
            }

            if (timer.getElapsedTimeSeconds() > 0.9) {
                if (transferStep == 2) {
                    robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_CLOSE);
                    ++transferStep;
                }

                if (timer.getElapsedTimeSeconds() > 1.3) {
                    if (transferStep == 3) {
                        robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                        transferStep = -1;
                    }

                    if (timer.getElapsedTimeSeconds() > 1.5) {
                        if (transferStep == 4) {
                            robot.intake.setPivot(UniversalValues.INTAKE_TRANSFER - 0.2);
                            //                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

                            ++transferStep;
                        }

                        if (timer.getElapsedTimeSeconds() > 1.9) {
                            if (transferStep == 5) {
                                robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

                                ++transferStep;
                            }

                            if (timer.getElapsedTimeSeconds() > 2.1) {
                                if (transferStep == 6) {
                                    robot.intake.setPivot(UniversalValues.INTAKE_TRANSFER);


                                    ++transferStep;
                                }

                                if (timer.getElapsedTimeSeconds() > 2.5) {
                                    if (transferStep == 7) {
                                        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);


                                        ++transferStep;
                                    }

                                    if (timer.getElapsedTimeSeconds() > 2.7) {
                                        if (transferStep == 8) {
                                            robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                                            transferStep++;
                                        }
                                    }

                                    if (timer.getElapsedTimeSeconds() > 2.9) {

                                        if (transferStep == 9) {
                                            robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
                                            robot.intake.setPivot(UniversalValues.INTAKE_INT);

                                            transferStep = -1;
                                            transferCompleted = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

    }
}
