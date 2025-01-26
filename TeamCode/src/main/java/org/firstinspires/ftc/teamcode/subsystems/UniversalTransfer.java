package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;

public class UniversalTransfer {
    private org.firstinspires.ftc.teamcode.subsystems.intake intake;
    private org.firstinspires.ftc.teamcode.subsystems.outtake outtake;
    private boolean transferCompleted = false;
    private boolean samplePickedUp = false;

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

//            intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
            intake.setClawPivot(universalValues.CLAW_VERTICAL);

            outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);
            outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);

            ++transferStep;
        }
        if (timer.getElapsedTimeSeconds() > 0.2) {
            intake.ManualLevel(universalValues.INTAKE_RETRACT, 0.6);

            if (transferStep == 1) {
                intake.setPivot(0.92);
                intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
                intake.OpenIntake(universalValues.CLAW_LOOSE);

                ++transferStep;
            }

            // TODO: make delay smaller

            if (timer.getElapsedTimeSeconds() > 0.8) {
                if (transferStep == 2) {
                   //outtake.CloseOuttake(universalValues.OUTTAKE_LOOSE);
                    samplePickedUp = true;
                    intake.CloseIntake(universalValues.CLAW_CLOSE);
                    ++transferStep;
                }

                if (timer.getElapsedTimeSeconds() > 1) {
                    if (transferStep == 3) {
                        intake.setPivot(universalValues.INTAKE_TRANSFER-0.2);
//                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

                        ++transferStep;
                    }

                    if (timer.getElapsedTimeSeconds() > 1.4) {
                            if (transferStep == 4) {
                                outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

                                ++transferStep;
                            }

                            if (timer.getElapsedTimeSeconds() > 1.6) {
                                if (transferStep == 5) {
                                    intake.setPivot(universalValues.INTAKE_TRANSFER);


                                    ++transferStep;
                                }

                                if (timer.getElapsedTimeSeconds() > 2) {
                                    if (transferStep == 6) {
                                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

                                        ++transferStep;
                                    }

                                    if (timer.getElapsedTimeSeconds() > 2.2) {
                                        if (transferStep == 7) {
                                            intake.CloseIntake(universalValues.CLAW_OPEN);
                                            outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);
                                            intake.setPivot(universalValues.INTAKE_INT);

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



    public void resetTransfer() {
        timerSingleton = true;
        transferStep = 0;
        transferCompleted = false;
        samplePickedUp = false;
    }

    public boolean isTransferCompleted() {
        return transferCompleted;
    }

    public boolean isSamplePickedUp() { return samplePickedUp; }
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