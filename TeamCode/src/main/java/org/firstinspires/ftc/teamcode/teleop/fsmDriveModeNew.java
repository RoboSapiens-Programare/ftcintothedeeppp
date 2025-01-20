package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_TIMER;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.CLAW_VERTICAL;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_INT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_CLIPON_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_CLIPON_UP;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_COLLECT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_COLLECT_NEW_TRANSFER;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_DUMP_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_EXTEND_MID;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.OUTTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.subsystems.universalValues.PIVOT_TIMER;
import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.robot;

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

    private enum IntakeState {
        INTAKE_START, INTAKE_CLAW_COLLECT_POSITION, INTAKE_RETRACT, INTAKE_EXTEND,
        OUTTAKE_MID, OUTTAKE_EXTEND, OUTTAKE_RETRACT, OUTTAKE_SAMPLE,
        TRANSFER
    }

    private double clawPivot = CLAW_HORIZONTAL;
    private IntakeState intakeState = IntakeState.INTAKE_START;

    private void initializeRobot() {
        robot.intake.ManualLevel(0, 1);
        robot.intake.CloseIntake(CLAW_CLOSE);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT);
        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
        robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
    }

    private void handleIntakeStart() {
        isPressed = false;
        isSquare = false;
        isMoving = false;
        if (isStarted) {
            robot.intake.setPivot(INTAKE_INT);
            isStarted = false;
        }
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.ManualLevel(INTAKE_EXTEND, 1);
            intakeState = IntakeState.INTAKE_EXTEND;
        }
        if (intakeTimer.seconds() > 0.2) {
            robot.intake.setPivot(INTAKE_INT);
            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
        }
        if (gamepad2.triangle) {
            robot.outtake.ManualLevel(OUTTAKE_EXTEND_MID, 1);
            intakeState = IntakeState.OUTTAKE_MID;
        }
        if (gamepad2.square) {
            robot.outtake.setPivot(OUTTAKE_CLIPON_UP);
            intakeTimer.reset();
            robot.outtake.OpenOuttake(OUTTAKE_CLOSE);
            intakeState = IntakeState.OUTTAKE_SAMPLE;
        }
    }

    private void handleIntakeExtend() {
        if (abs(robot.intake.intakeMotor.getCurrentPosition() - INTAKE_EXTEND) < 10) {
            robot.intake.setPivot(INTAKE_DOWN);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            robot.intake.OpenIntake(CLAW_OPEN - 0.185);
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
            robot.intake.CloseIntake(CLAW_CLOSE);
        } else if (gamepad1.right_bumper) {
            robot.intake.OpenIntake(CLAW_OPEN - 0.185);
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
            robot.intake.setPivot(INTAKE_INT);
        }
        if(gamepad1.square){
            robot.intake.setPivot(INTAKE_DOWN);
        }
    }

    private void handleTransfer() {
        robot.universalTransfer.transfer();

        telemetry.addData("transfer completed", robot.universalTransfer.isTransferCompleted());
        if (!robot.universalTransfer.isTransferCompleted()){
            return;
        }

        intakeState = IntakeState.OUTTAKE_MID;
    }

    private void handleOuttakeMid() {
        if (gamepad2.cross) {
            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.4);
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
            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.4);
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
        intakeState = IntakeState.INTAKE_START;
    }

    private void handleOuttakeSample() {
        if(gamepad1.square && !isSquare) {
            isSquare = true;
            isTimer = true;
        }
        if(abs(gamepad1.left_stick_y) > 0.02 && isSquare && !isMoving){
            isMoving = true;
        }
        if(isSquare && isMoving){
            if(isTimer){
                intakeTimer.reset();
                isTimer = false;
            }
            robot.outtake.setPivot(OUTTAKE_CLIPON_DOWN);
            if(intakeTimer.seconds() > 0.3){
                robot.outtake.OpenOuttake(OUTTAKE_OPEN);

                if(intakeTimer.seconds() > 0.5) {
                    robot.outtake.setPivot(OUTTAKE_CLIPON_UP);
                    robot.outtake.setPivot(OUTTAKE_COLLECT);
                    intakeState = IntakeState.INTAKE_START;
                    intakeTimer.reset();
                }
            }
        }
    }

    private void updateFollower(double power) {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.setMaxPower(power);
        follower.update();
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

        DcMotorEx leftRear;
        DcMotorEx rightRear;
        DcMotorEx leftFront;
        DcMotorEx rightFront;

        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        telemetry.addData("valoare glisiera", robot.intake.intakeMotor.getCurrentPosition());
        telemetry.addData("State", intakeState);

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
}