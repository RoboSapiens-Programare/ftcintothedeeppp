package org.firstinspires.ftc.teamcode.subsystems.tests;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

//@Disabled
@Autonomous(name = "Transfer Test", group = "Autonomous")
public class TransferTest extends OpMode {
    private Robot robot;
    private Timer actionTimer;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        actionTimer = new Timer();

        robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
        robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND,0.75);
        robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
        robot.intake.setPivot(UniversalValues.INTAKE_INT);
        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
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
