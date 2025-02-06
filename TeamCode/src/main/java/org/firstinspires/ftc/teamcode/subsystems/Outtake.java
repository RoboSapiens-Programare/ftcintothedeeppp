package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Outtake {
    public Servo pivotOut1, pivotOut2, outtake;
//    public DistanceSensor outtakeSensor;
    public DcMotorEx outtakeMotor, outtakeMotor2;

    public TouchSensor outtakeSensor;

    public Outtake(HardwareMap hardwareMap){
        pivotOut1 = hardwareMap.get(Servo.class, "pivotOut1");
        pivotOut2 = hardwareMap.get(Servo.class, "pivotOut2");
        outtake = hardwareMap.get(Servo.class, "outtake");

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        outtakeSensor = hardwareMap.get(TouchSensor.class, "outtakeSensor");

        outtake.setDirection(Servo.Direction.REVERSE);

        pivotOut1.setDirection(Servo.Direction.FORWARD);
        pivotOut2.setDirection(Servo.Direction.REVERSE);

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void setPivot(double position){
        pivotOut1.setPosition(position);
        pivotOut2.setPosition(position);
    }

    public void ManualLevel(int ManualTarget, double power) {
        outtakeMotor.setTargetPosition(ManualTarget);
        outtakeMotor2.setTargetPosition(ManualTarget);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(outtakeMotor.getCurrentPosition() > ManualTarget && outtakeMotor2.getCurrentPosition() > ManualTarget) {
            outtakeMotor.setPower(power);
            outtakeMotor2.setPower(power);

        } else if (!outtakeSensor.isPressed() && outtakeMotor.getCurrentPosition() < ManualTarget && outtakeMotor2.getCurrentPosition() < ManualTarget) {
            outtakeMotor.setPower(-power);
            outtakeMotor2.setPower(-power);

        } else {
            outtakeMotor.setPower(-power);
            outtakeMotor2.setPower(-power);

        }
    }

    public void OpenOuttake(double position){
        outtake.setPosition(position);
    }

    public void CloseOuttake(double position){
        outtake.setPosition(position);
    }

    public void DisableServos() {
        pivotOut1.getController().pwmDisable();
        pivotOut2.getController().pwmDisable();
        outtake.getController().pwmDisable();
    }

    public void EnableServos() {
        pivotOut1.getController().pwmEnable();
        pivotOut2.getController().pwmEnable();
        outtake.getController().pwmEnable();
    }


}
