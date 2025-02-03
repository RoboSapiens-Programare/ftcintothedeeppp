package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;

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

//            intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
            intake.setClawPivot(UniversalValues.CLAW_VERTICAL);

            outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
            outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);

            ++transferStep;
        }
        if (timer.getElapsedTimeSeconds() > 0.4) {
            intake.ManualLevel(UniversalValues.INTAKE_RETRACT, 0.6);

            if (transferStep == 1) {
                intake.setPivot(1);
                intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);

                ++transferStep;
            }

            if (timer.getElapsedTimeSeconds() > 0.7) {
                if (transferStep == 2) {
                    intake.OpenIntake(UniversalValues.CLAW_LOOSE);
                    ++transferStep;
                }

                if (timer.getElapsedTimeSeconds() > 1.1) {
                    if (transferStep == 3) {
                        samplePickedUp = true;
                        intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                        ++transferStep;
                    }

                    if (timer.getElapsedTimeSeconds() > 1.3) {
                        if (transferStep == 4) {
                            intake.setPivot(UniversalValues.INTAKE_TRANSFER - 0.2);
                            //                        outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

                            ++transferStep;
                        }

                        if (timer.getElapsedTimeSeconds() > 1.7) {
                            if (transferStep == 5) {
                                outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

                                ++transferStep;
                            }

                            if (timer.getElapsedTimeSeconds() > 1.9) {
                                if (transferStep == 6) {
                                    intake.setPivot(UniversalValues.INTAKE_TRANSFER);


                                    ++transferStep;
                                }

                                if (timer.getElapsedTimeSeconds() > 2.3) {
                                    if (transferStep == 7) {
                                        outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);


                                        ++transferStep;
                                    }

                                    if (timer.getElapsedTimeSeconds() > 2.5) {
                                        if (transferStep == 8) {
                                            intake.CloseIntake(UniversalValues.CLAW_OPEN);
                                            transferStep++;
                                        }
                                    }

                                    if (timer.getElapsedTimeSeconds() > 2.7) {

                                        if (transferStep == 9) {
                                            outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
                                            intake.setPivot(UniversalValues.INTAKE_INT);

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