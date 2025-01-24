package org.firstinspires.ftc.teamcode.teleop;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_VERTICAL;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_INT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_TRANSFER;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_COLLECT_NEW_TRANSFER;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_DUMP_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_EXTEND_MID;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_EXTEND_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.PIVOT_TIMER;
import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "FSM DRIVE MODE NEW", group = "FSMTELEOP")
public class fsmDriveModeNew extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot;
    private Follower follower;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean isHorizontal = true;
    private boolean isStarted = true;
    private boolean isPressed = false;
    private boolean isSquare = false;
    private boolean isTimer = true;
    private boolean isMoving = false;

    private boolean wallCollect = false;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private enum IntakeState {
        INTAKE_START, INTAKE_CLAW_COLLECT_POSITION,
        INTAKE_PRE_COLLECT_COMPACT, INTAKE_COLLECT_COMPACT,
        INTAKE_EXTEND, OUTTAKE_MID, OUTTAKE_EXTEND, OUTTAKE_RETRACT, OUTTAKE_SAMPLE,
        TRANSFER
    }

    private double clawPivot = CLAW_HORIZONTAL;
    private IntakeState intakeState = IntakeState.INTAKE_START;

    private void initializeRobot() {
        robot.intake.ManualLevel(INTAKE_RETRACT, 1);
        robot.intake.CloseIntake(CLAW_CLOSE);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT);
        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
    }

    private void handleIntakeStart() {
        robot.universalTransfer.resetTransfer();
        robot.intake.ManualLevel(INTAKE_RETRACT, 0.5);
        robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
        isPressed = false;
        isSquare = false;
        isMoving = false;
        if (isStarted) {
            robot.intake.setPivot(INTAKE_INT);
            isStarted = false;
        }
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.ManualLevel(INTAKE_EXTEND, 0.75);
            intakeState = IntakeState.INTAKE_EXTEND;
        }
        if (intakeTimer.seconds() > 0.2) {
            robot.intake.setPivot(INTAKE_INT);
            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
        }
        if (gamepad2.triangle) {
            robot.outtake.ManualLevel(OUTTAKE_EXTEND_MID, 0.75);
            outtakeTimer.reset();
            intakeState = IntakeState.OUTTAKE_MID;
        }
        if (gamepad1.square) {
            intakeTimer.reset();
            outtakeTimer.reset();
            intakeState = IntakeState.OUTTAKE_SAMPLE;
        }

        if (gamepad1.circle) {
            intakeState = IntakeState.INTAKE_PRE_COLLECT_COMPACT;
        }
    }

    private void handleIntakeExtend() {
        if (abs(robot.intake.intakeMotor.getCurrentPosition() - INTAKE_EXTEND) < 10) {
            robot.intake.setPivot(INTAKE_DOWN);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            robot.intake.OpenIntake(CLAW_OPEN);
            intakeTimer.reset();
            intakeState = IntakeState.INTAKE_CLAW_COLLECT_POSITION;
        }
    }

    private void adjustClawPivot() {
        if (gamepad1.dpad_right && outtakeTimer.seconds() > PIVOT_TIMER && clawPivot < 1) {
            clawPivot += 0.01;
            robot.intake.setClawPivot(clawPivot);
            outtakeTimer.reset();
        } else if (gamepad1.dpad_left && outtakeTimer.seconds() > PIVOT_TIMER && clawPivot > CLAW_VERTICAL) {
            clawPivot -= 0.01;
            robot.intake.setClawPivot(clawPivot);
            outtakeTimer.reset();
        }
    }

    private void adjustClawPosition() {
        if (gamepad1.dpad_down) {
            robot.intake.setClawPivot(CLAW_VERTICAL);
            clawPivot = CLAW_VERTICAL;
            isHorizontal = false;
        } else if (gamepad1.dpad_up) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            isHorizontal = true;
        }
    }

    private void handleIntakeClawCollectPosition() {
        if (gamepad1.left_bumper) {
            if (wallCollect)
                robot.intake.ManualLevel(INTAKE_EXTEND-50, 0.8);
            robot.intake.CloseIntake(CLAW_CLOSE);
        } else if (gamepad1.right_bumper) {
            if (wallCollect)
                robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
            robot.intake.OpenIntake(CLAW_OPEN);
        }

        adjustClawPivot();
        adjustClawPosition();

        if (gamepad1.left_trigger > 0.1) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;

            robot.intake.setPivot(INTAKE_INT);
            intakeState = IntakeState.TRANSFER;
        }
        if(gamepad1.cross){
            robot.intake.setPivot(INTAKE_DOWN);
            wallCollect = false;
        }
        if(gamepad1.square){
            robot.intake.setPivot(INTAKE_INT);
            wallCollect = true;
        }
        if (gamepad1.triangle) {
            robot.intake.setPivot(INTAKE_TRANSFER);
            wallCollect = false;
        }
    }

    private void handleIntakePreCompactCollect() {
        robot.intake.ManualLevel(INTAKE_RETRACT+50, 0.8);
        robot.intake.setPivot(INTAKE_INT);
        wallCollect = true;

        robot.intake.OpenIntake(CLAW_OPEN);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        clawPivot = CLAW_HORIZONTAL;

        intakeState = IntakeState.INTAKE_COLLECT_COMPACT;
    }

    private void handleIntakeCompactCollect() {
        if (gamepad1.left_bumper) {
            if (wallCollect)
                robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
            robot.intake.CloseIntake(CLAW_CLOSE);
        } else if (gamepad1.right_bumper) {
            if (wallCollect)
                robot.intake.ManualLevel(INTAKE_RETRACT+50, 0.8);
            robot.intake.OpenIntake(CLAW_OPEN);
        }

        adjustClawPivot();
        adjustClawPosition();

        if (gamepad1.left_trigger > 0.1) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;

            robot.intake.setPivot(INTAKE_INT);
            intakeState = IntakeState.TRANSFER;
        }

        if(gamepad1.cross){
            robot.intake.setPivot(INTAKE_DOWN);
            wallCollect = false;
        }
        if(gamepad1.square){
            robot.intake.setPivot(INTAKE_INT);
            wallCollect = true;
        }
        if (gamepad1.triangle) {
            robot.intake.setPivot(INTAKE_TRANSFER);
            wallCollect = false;
        }

        if (gamepad1.circle) {
            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
            robot.intake.OpenIntake(CLAW_OPEN);
            robot.intake.setPivot(INTAKE_INT);

            intakeState = IntakeState.INTAKE_START;
        }
    }

    private void handleTransfer() {
        robot.universalTransfer.transfer();

        telemetry.addData("transfer completed", robot.universalTransfer.isTransferCompleted());
        if (!robot.universalTransfer.isTransferCompleted()){
            return;
        }

        intakeState = IntakeState.INTAKE_START;
    }

    private void handleOuttakeMid() {
        if (gamepad2.cross) {
            outtakeTimer.reset();
            intakeState = IntakeState.OUTTAKE_RETRACT;
        }
        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
        if (gamepad2.right_bumper) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        }
        if (abs(robot.outtake.outtakeMotor.getCurrentPosition() - OUTTAKE_EXTEND_MID) < 100 && gamepad2.triangle) {
            robot.outtake.ManualLevel(OUTTAKE_EXTEND, 1);
            intakeState = IntakeState.OUTTAKE_EXTEND;
        }
    }

    private void handleOuttakeExtend() {
        robot.universalTransfer.resetTransfer();

        if (gamepad2.cross) {

            intakeState = IntakeState.OUTTAKE_RETRACT;
        }
        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
        if (gamepad2.right_bumper) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        }
    }

    private void handleOuttakeRetract() {

        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        if (outtakeTimer.seconds()>0.35)
        {
            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.4);
            intakeState = IntakeState.INTAKE_START;
        }
    }

    private void handleOuttakeSample() {
        robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
        if (outtakeTimer.seconds()>0.4)
        {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
            if (outtakeTimer.seconds()>0.6)
            {
                robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                intakeState=IntakeState.INTAKE_START;
            }
        }
    }

    private void updateFollower(double power) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    @Override
    public void init() {
        robot = new robot(hardwareMap);
        follower = new Follower(hardwareMap);
        intakeTimer.reset();
        outtakeTimer.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        follower.startTeleopDrive();
        initializeRobot();

        Constants.setConstants(FConstants.class, LConstants.class);


        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }

    @Override
    public void loop() {
        telemetry.addData("valoare glisiera", robot.intake.intakeMotor.getCurrentPosition());
        telemetry.addData("State", intakeState);
        telemetry.addData("left joystick x", gamepad1.left_stick_x);
        telemetry.addData("left joystick y", gamepad1.left_stick_y);
        telemetry.addData("right joystick x", gamepad1.right_stick_x);
        telemetry.addData("right joystick y", gamepad1.right_stick_y);

        switch (intakeState) {

            case INTAKE_START:
                handleIntakeStart();
                break;
            case INTAKE_EXTEND:
                handleIntakeExtend();
                break;
            case INTAKE_CLAW_COLLECT_POSITION:
                handleIntakeClawCollectPosition();
                break;
            case INTAKE_PRE_COLLECT_COMPACT:
                handleIntakePreCompactCollect();
                break;
            case INTAKE_COLLECT_COMPACT:
                handleIntakeCompactCollect();
                break;
            case TRANSFER:
                handleTransfer();
                break;
            case OUTTAKE_MID:
                handleOuttakeMid();
                break;
            case OUTTAKE_EXTEND:
                handleOuttakeExtend();
                break;
            case OUTTAKE_RETRACT:
                handleOuttakeRetract();
                break;
            case OUTTAKE_SAMPLE:
                handleOuttakeSample();
                break;
            default:
                intakeState = IntakeState.INTAKE_START;
                break;
        }
        if(intakeState == IntakeState.OUTTAKE_SAMPLE) {
            updateFollower(0.4);
        }
        else updateFollower(1);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
        robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
    }
}