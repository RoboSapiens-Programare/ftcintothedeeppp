package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake {
    public Servo pivotin, pivotin2, pivotIntake, intake;

    public DcMotorEx intakeMotor, intakeMotor2;

    public TouchSensor intakeLimit;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(Servo.class, "intake");

        pivotin = hardwareMap.get(Servo.class, "pivotin");
        pivotin2 = hardwareMap.get(Servo.class, "pivotin2");
        pivotIntake = hardwareMap.get(Servo.class, "pivotIntake");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeLimit = hardwareMap.get(TouchSensor.class, "intakeLimit");

        intake.setDirection(Servo.Direction.FORWARD);

        pivotin.setDirection(Servo.Direction.FORWARD);
        pivotin2.setDirection(Servo.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setPivot(double position) {
        pivotin.setPosition(position);
        pivotin2.setPosition(position);
    }

    public void ManualLevel(int ManualTarget, double power) {
        intakeMotor.setTargetPosition(ManualTarget);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (intakeMotor.getCurrentPosition() < ManualTarget) {
            intakeMotor.setPower(power);
        } else if (!intakeLimit.isPressed() && intakeMotor.getCurrentPosition() > ManualTarget) {
            intakeMotor.setPower(-power);
        } else {
            intakeMotor.setPower(0);
            intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    public void OpenIntake(double position) {
        intake.setPosition(position);
    }

    public void CloseIntake(double position) {
        intake.setPosition(position);

    }

    public void setClawPivot(double position){
        pivotIntake.setPosition(position);
    }


}