package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

@Autonomous(name = "Claw Test", group = "Subsystem Tests")
public class clawTest extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot;

    @Override
    public void init() {
        robot = new robot(hardwareMap);

        robot.intake.ManualLevel(universalValues.INTAKE_EXTEND,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(universalValues.INTAKE_INT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
    }

    @Override
    public void loop() {
        robot.intake.OpenIntake(universalValues.CLAW_CLOSE);

    }
}
