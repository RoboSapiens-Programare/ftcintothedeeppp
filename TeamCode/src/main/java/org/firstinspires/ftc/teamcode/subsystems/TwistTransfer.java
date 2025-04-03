package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;

public class TwistTransfer {
    private Intake intake;
    private Outtake outtake;
    private boolean transferCompleted = false;
    private boolean samplePickedUp = false;

    private Timer timer;
    private boolean timerSingleton = true;
    private int transferStep =  0;
    private Timer barPickupTimer;
    private int barPickupStep = 0;

    public TwistTransfer(Intake intake,
                             Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
        timer = new Timer();
        barPickupTimer = new Timer();
    }

    private boolean barPickup() {
        if (barPickupStep == 0) {
            intake.setPivot((UniversalValues.INTAKE_INT+UniversalValues.INTAKE_UP)/2);

            ++barPickupStep;
            return false;
        }

        if (barPickupStep == 1 && barPickupTimer.getElapsedTimeSeconds() > 0.3) {
            intake.setPivot(UniversalValues.INTAKE_INT);

            ++barPickupStep;
            return  false;
        }

        if (barPickupStep == 2 && barPickupTimer.getElapsedTimeSeconds() > 0.6) {
            barPickupStep = 0;
            return  true;
        }

        return  false;
    }

    public void transfer() {
        if (timerSingleton) {
            timerSingleton = false;
            timer.resetTimer();
            barPickupTimer.resetTimer();
        }

        if (transferStep == 0) {

            intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
            outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
            intake.OpenIntake(UniversalValues.CLAW_LOOSE);
            intake.ManualLevel(UniversalValues.INTAKE_RETRACT, 0.6);
            intake.setPivot(UniversalValues.INTAKE_INT);

            outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

            if (this.barPickup())
                ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 0.4 && transferStep == 1) {
            samplePickedUp = true;
            intake.CloseIntake(UniversalValues.CLAW_CLOSE);
            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 0.6 && transferStep == 2) {
            intake.setClawPivot(UniversalValues.CLAW_FLIPPED);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 1.1 && transferStep == 3) {
            intake.setPivot(UniversalValues.INTAKE_TRANSFER);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 1.4 && transferStep == 4) {
            outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);

            ++transferStep;
        }

        if (timer.getElapsedTimeSeconds() > 1.6 && transferStep == 5) {
            intake.CloseIntake(UniversalValues.CLAW_OPEN);
            transferStep++;
        }

        if (timer.getElapsedTimeSeconds() > 1.8 && transferStep == 6) {

            outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
            intake.setPivot(UniversalValues.INTAKE_INT);
            intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);

            transferStep = -1;
            transferCompleted = true;
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