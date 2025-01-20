package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;

public class UniversalTransfer {
    private org.firstinspires.ftc.teamcode.subsystems.intake intake;
    private org.firstinspires.ftc.teamcode.subsystems.outtake outtake;
    private boolean transferCompleted = false;

    private Timer timer;
    private boolean timerSingleton = true;
    private int transferStep =  0;

    public UniversalTransfer(org.firstinspires.ftc.teamcode.subsystems.intake intake,
                             org.firstinspires.ftc.teamcode.subsystems.outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
        timer = new Timer();
    }

    // do the transfer from when the sliders are fully extended, the pivot in
    public void transfer() {
        if (timerSingleton) {
            timerSingleton = false;
            timer.resetTimer();
        }

        if (transferStep == 0) {
            intake.setPivot(universalValues.INTAKE_INT);
            intake.OpenIntake(universalValues.CLAW_LOOSE);
            intake.setClawPivot(universalValues.CLAW_HORIZONTAL);

            outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
            outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);

            ++transferStep;
        }
        if (timer.getElapsedTimeSeconds() > 0.2) {
            if (transferStep == 1) {
                intake.ManualLevel(universalValues.INTAKE_RETRACT, 0.8);
                intake.setPivot(universalValues.INTAKE_TRANSFER);

                ++transferStep;
            }

            // TODO: make delay smaller

            if (timer.getElapsedTimeSeconds() > 1.4) {
                if (transferStep == 2) {
                    outtake.CloseOuttake(universalValues.OUTTAKE_LOOSE);
                    intake.CloseIntake(universalValues.CLAW_LOOSE);
                    ++transferStep;
                }

                if (timer.getElapsedTimeSeconds() > 1.55) {
                    if (transferStep == 3) {
                        intake.OpenIntake(universalValues.CLAW_OPEN);
                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

                        ++transferStep;
                    }

                    if (timer.getElapsedTimeSeconds() > 1.7) {
                            if (transferStep == 4) {
                                outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);
                                ++transferStep;
                            }

                            if (timer.getElapsedTimeSeconds() > 1.95) {
                                if (transferStep == 5) {
                                    outtake.OpenOuttake(universalValues.OUTTAKE_LOOSE);
                                    intake.setPivot(universalValues.INTAKE_INT);

                                    ++transferStep;
                                }

                                if (timer.getElapsedTimeSeconds() > 2.45) {
                                    if (transferStep == 6) {
                                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

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



    public void resetTransfer() {
        timerSingleton = true;
        transferStep = 0;
        transferCompleted = false;
    }

    public boolean isTransferCompleted() {
        return transferCompleted;
    }
}

/* USAGE:
 * public void loop() {
 *     ...
 *     robot.transfer();
 *
 *     while (!isTransferCompleted())
 *         return; // or any kind of skip
 *     resetTransfer();
 *     ...
 * }
 */