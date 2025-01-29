package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

@Autonomous(name = "Transfer Test", group = "Subsystem Tests")
public class TransferTest extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot;
    private Timer actionTimer;

    @Override
    public void init() {
        robot = new robot(hardwareMap);
        actionTimer = new Timer();

        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.ManualLevel(universalValues.INTAKE_EXTEND,0.75);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setPivot(universalValues.INTAKE_INT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
    }

    @Override
    public void loop() {
        robot.universalTransfer.transfer();
        telemetry.addData("servo1 pos", robot.intake.pivotin.getPosition());
        telemetry.addData("servo2 pos", robot.intake.pivotin2.getPosition());
//        telemetry.addData("transfer step", robot.universalTransfer.transferStep);
        telemetry.update();
    }
}
