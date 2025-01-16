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

            outtake.setPivot(universalValues.OUTTAKE_COLLECT);
            outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);

            ++transferStep;
        }
        if (timer.getElapsedTimeSeconds() > 0.2) {
            if (transferStep == 1) {
                intake.ManualLevel(universalValues.INTAKE_RETRACT, 1);
                intake.setPivot(universalValues.INTAKE_TRANSFER);

                ++transferStep;
            }

            // TODO: make delay smaller
            if (timer.getElapsedTimeSeconds() > 3) {
                if (transferStep == 2) {
                    intake.OpenIntake(universalValues.CLAW_CLOSE);
                    intake.ManualLevel(universalValues.INTAKE_EXTEND/2, 1);

                    ++transferStep;
                }

                if (timer.getElapsedTimeSeconds() > 4.5) {
                    if (transferStep == 3) {
                        outtake.setPivot(universalValues.OUTTAKE_COLLECT);
                        intake.setPivot(universalValues.INTAKE_TRANSFER);

                        ++transferStep;
                    }

                    if (timer.getElapsedTimeSeconds() > 5) {
                        if (transferStep == 4) {
                            intake.ManualLevel(universalValues.INTAKE_RETRACT, 1);

                            ++transferStep;
                        }

                        if (timer.getElapsedTimeSeconds() > 6) {
                            if (transferStep == 5) {
                                outtake.CloseOuttake(universalValues.OUTTAKE_LOOSE);

                                ++transferStep;
                            }
                            if (timer.getElapsedTimeSeconds() > 6.3) {
                                if (transferStep == 6) {
                                    intake.OpenIntake(universalValues.CLAW_LOOSE);

                                    ++transferStep;
                                }
                                if (timer.getElapsedTimeSeconds() > 6.8) {
                                    if (transferStep == 7) {
                                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

                                        ++transferStep;
                                    }

                                    if (timer.getElapsedTimeSeconds() > 6.7) {
                                        if (transferStep == 8) {
                                            intake.OpenIntake(universalValues.CLAW_OPEN);

                                            // TODO: Check value
                                            outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);

                                            ++transferStep;
                                        }

                                        if (timer.getElapsedTimeSeconds() > 7.2) {
                                            if (transferStep == 9) {
                                                outtake.OpenOuttake(universalValues.CLAW_LOOSE);
                                                intake.setPivot(universalValues.INTAKE_INT);
                                            }
                                            if (timer.getElapsedTimeSeconds() > 7.5) {
                                                outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
                                                transferStep = -1; // STOP executing transfer steps

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