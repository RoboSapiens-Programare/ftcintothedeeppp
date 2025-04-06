package org.firstinspires.ftc.teamcode.teleop;


import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_VERTICAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_TRANSFER;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_ASCENT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_DUMP_BUCKET;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_DUMP_BUCKET_DIAG;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND_ASCENT_INTERMEDIARY;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND_GRAB;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND_MID;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND_SPECIMEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_GRAB_BAR;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.PIVOT_TIMER;
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
import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "fake FSM", group = "0. TeleOp")
public class fakeFSM extends OpMode {
    private Robot robot;
    private Follower follower;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private final ElapsedTime speedTimer = new ElapsedTime();

    private boolean isStarted = true;

    private int ascentStep = 0;

    private boolean wallCollect = false;
    private boolean doAscent = false;
    private boolean noTransfer = false;
    private boolean isIntakeMoving = false;
    private boolean intakeFixSingleton = true;
    private boolean outtakeFixSingleton = true;
    private boolean intakeMoveFixSingleton = true;
    private boolean fixExtensionSingleton = true;

    private boolean outtakeHoming = false;

    private boolean toggleSpeed = false;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;


    private boolean mode = false;
    private double modeDelay = 15;
    private final ElapsedTime modeTimer = new ElapsedTime();
    private final ElapsedTime modeUpdateTimer = new ElapsedTime();
    private double modeAlpha = 0;
    private double modePower = 0.3F;


    private enum IntakeState {
        INTAKE_START, INTAKE_CLAW_COLLECT_POSITION,
        INTAKE_EXTEND, OUTTAKE_MID, OUTTAKE_EXTEND, OUTTAKE_RETRACT, OUTTAKE_SAMPLE,
        TRANSFER, ASCENT
    }

    private double clawPivot = CLAW_HORIZONTAL;
    private IntakeState intakeState = IntakeState.INTAKE_START;

    private void initializeRobot() {
        robot.intake.ManualLevel(INTAKE_RETRACT, 1);
        robot.outtake.ManualLevel(0, 1);
        robot.intake.CloseIntake(CLAW_OPEN);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT);
        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
    }

    private void handleIntakeStart() {
        telemetry.addData("OUTTAKE_EXTEND", OUTTAKE_EXTEND);
        telemetry.addData("OUTTAKE_EXTEND_SPECIMEN", OUTTAKE_EXTEND_SPECIMEN);
        telemetry.addData("OUTTAKE_RETRACT", OUTTAKE_RETRACT);

        telemetry.addData("INTAKE_EXTEND", INTAKE_EXTEND);
        telemetry.addData("INTAKE_RETRACT", INTAKE_RETRACT);

        robot.universalTransfer.resetTransfer();
        if (!isIntakeMoving) {
            telemetry.addData("RETRACTING INTAKE", "true");
            robot.intake.ManualLevel(INTAKE_RETRACT, 0.5);
        }
        if (!outtakeHoming) {
            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
        }
        if (isStarted) {
            robot.intake.setPivot(INTAKE_INT);
            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
            isStarted = false;
        }
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
            intakeState = IntakeState.INTAKE_EXTEND;
            noTransfer = true;
        }
        if (gamepad2.triangle) {
            robot.outtake.ManualLevel(OUTTAKE_EXTEND_MID, 0.75);
            outtakeTimer.reset();
            intakeState = IntakeState.OUTTAKE_MID;
        }
        if (gamepad1.circle) {
            intakeTimer.reset();
            outtakeTimer.reset();
            intakeState = IntakeState.OUTTAKE_SAMPLE;
        }

        if (gamepad2.dpad_left || !intakeFixSingleton) {
            if (robot.intake.intakeLimit.isPressed()) {
                INTAKE_RETRACT = robot.intake.intakeMotor.getCurrentPosition();
                INTAKE_EXTEND = INTAKE_RETRACT + 290;
                isIntakeMoving = false;
                robot.intake.ManualLevel(INTAKE_RETRACT,0.8);
                intakeFixSingleton = false;
                robot.intake.intakeMotor.setTargetPosition(robot.intake.intakeMotor.getCurrentPosition());
            }

            if (!isIntakeMoving && !robot.intake.intakeLimit.isPressed() && intakeFixSingleton) {
                robot.intake.intakeMotor.setPower(-0.2);
                robot.intake.intakeMotor.setTargetPosition(robot.intake.intakeMotor.getCurrentPosition() - 1000);
                isIntakeMoving = true;
            }
        }

        if (gamepad2.dpad_right) {
            intakeFixSingleton = true;
        }

        // EMERGENCY PARK
        if (gamepad2.left_trigger > 0.1) {
            robot.outtake.setPivot(0.9);
        }
        if (gamepad2.right_trigger > 0.1) {
            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
        }

        // lvl 1 ascent
        if (gamepad2.left_bumper) {
            robot.outtake.setPivot(OUTTAKE_GRAB_BAR);
            robot.outtake.ManualLevel(OUTTAKE_EXTEND_ASCENT_INTERMEDIARY, 0.8);

            intakeState = IntakeState.ASCENT;
        }


        // compact collect
        if (gamepad1.left_bumper) {
            robot.intake.CloseIntake(CLAW_CLOSE);
        } else if (gamepad1.right_bumper) {
            robot.intake.OpenIntake(CLAW_OPEN);
        }

        adjustClawPivot();
        adjustClawPosition();

        if (gamepad1.left_trigger > 0.1 && !noTransfer) {
            robot.intake.setClawPivot(0.6);
            clawPivot = 0.6;

            robot.intake.setPivot(INTAKE_INT);
            intakeState = IntakeState.TRANSFER;
        }


        if(gamepad1.square){
            robot.intake.setPivot(INTAKE_INT);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            wallCollect = true;
            noTransfer = false;
        }
        if (gamepad1.triangle) {
            robot.intake.setPivot(INTAKE_TRANSFER);
            wallCollect = false;
        }
    }

    private void handleIntakeExtend() {
        if (abs(robot.intake.intakeMotor.getCurrentPosition() - INTAKE_EXTEND) < 50) {
            robot.intake.setPivot(INTAKE_DOWN);
            wallCollect = false;
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            robot.intake.OpenIntake(CLAW_OPEN);
            intakeTimer.reset();
            intakeState = IntakeState.INTAKE_CLAW_COLLECT_POSITION;
        }
    }

    private void adjustClawPivot() {
        if (gamepad1.dpad_right && outtakeTimer.seconds() > PIVOT_TIMER && clawPivot > CLAW_VERTICAL) {
            clawPivot -= 0.01;
            robot.intake.setClawPivot(clawPivot);
            outtakeTimer.reset();
        } else if (gamepad1.dpad_left && outtakeTimer.seconds() > PIVOT_TIMER && clawPivot < 1) {
            clawPivot += 0.01;
            robot.intake.setClawPivot(clawPivot);
            outtakeTimer.reset();
        }
    }

    private void adjustClawPosition() {
        if (gamepad1.dpad_down) {
            robot.intake.setClawPivot(CLAW_VERTICAL);
            clawPivot = CLAW_VERTICAL;
        } else if (gamepad1.dpad_up) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
        }
    }

    private void handleIntakeClawCollectPosition() {
        telemetry.addData("fixExtensionSingleton", fixExtensionSingleton);
        telemetry.addData("intakeTimer", intakeTimer.seconds());

        if (gamepad2.left_bumper) {
            robot.outtake.setPivot(OUTTAKE_GRAB_BAR);
            robot.outtake.ManualLevel(OUTTAKE_EXTEND_ASCENT_INTERMEDIARY, 0.8);

            intakeState = IntakeState.ASCENT;
        }

        if (gamepad1.left_bumper) {
//            if (wallCollect)
//                robot.intake.ManualLevel(INTAKE_EXTEND-50, 0.8);
            robot.intake.CloseIntake(CLAW_CLOSE);
            intakeMoveFixSingleton = true;

        } else if (gamepad1.right_bumper || !intakeMoveFixSingleton) {

            intakeMoveFixSingleton = false;
            if (wallCollect) {
                robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
            }

            if (intakeTimer.seconds() > 0.25 || !wallCollect) {
                robot.intake.CloseIntake(CLAW_OPEN);
                intakeMoveFixSingleton = true;
            }

        } else if (intakeMoveFixSingleton && fixExtensionSingleton) {
            intakeTimer.reset();
        }

        adjustClawPivot();
        adjustClawPosition();

        if (gamepad1.left_trigger > 0.1 && !noTransfer) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;

            robot.intake.setPivot(INTAKE_INT);
            intakeState = IntakeState.TRANSFER;
        }
        if(gamepad1.cross || !fixExtensionSingleton || gamepad1.right_trigger > 0.1){
            if (fixExtensionSingleton) {
                robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                intakeTimer.reset();
                fixExtensionSingleton = false;
            }

            if (intakeTimer.seconds() > 0.25) {
                robot.intake.setPivot(INTAKE_DOWN);
                fixExtensionSingleton = true;
            }

            wallCollect = false;
        }
        if(gamepad1.square){
            robot.intake.setPivot(INTAKE_INT);
            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            wallCollect = true;
            noTransfer = false;
        }
        if (gamepad1.triangle) {
            robot.intake.setPivot(INTAKE_TRANSFER);
            wallCollect = false;
        }

        if (gamepad1.circle) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;

            robot.intake.setPivot(INTAKE_INT);

            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
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
        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET_DIAG);
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
        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET_DIAG);
        if (gamepad2.right_bumper) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        }
    }

    private void handleOuttakeRetract() {
        if (outtakeFixSingleton) {
            outtakeFixSingleton = false;

            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);

            robot.outtake.outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.outtake.outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.outtake.outtakeMotor.setPower(0.4);
            robot.outtake.outtakeMotor2.setPower(0.4);

            outtakeHoming = true;
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
                robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
                robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                intakeState= IntakeState.INTAKE_START;
            }
        }
    }

    private void handleAscent() {
        if (gamepad2.right_bumper) {
            intakeState = IntakeState.INTAKE_START;
            ascentStep = 0;
            return;
        }

        if (gamepad2.dpad_up) {
            doAscent = true;
        }

        if (gamepad2.dpad_down) {
            doAscent = false;
        }

        if (doAscent) {
            if (ascentStep == 0) {
                robot.outtake.ManualLevel(OUTTAKE_EXTEND_GRAB, 0.8);
                outtakeTimer.reset();
                ++ascentStep;
            }

            if (ascentStep == 1 && outtakeTimer.seconds() > 1) {
                robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
                ++ascentStep;
            }

            if (ascentStep == 2 && outtakeTimer.seconds() > 1.35) {
                robot.outtake.ManualLevel(OUTTAKE_ASCENT, 1);
                robot.outtake.DisableServos();

                ++ascentStep;
            }
        }
    }

    private void outtakeHomingCheck() {
        if (robot.outtake.outtakeSensor.isPressed() && outtakeHoming) {
            robot.outtake.outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.outtake.outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            OUTTAKE_RETRACT = -15;
            OUTTAKE_EXTEND_MID = OUTTAKE_RETRACT -1770;
            OUTTAKE_EXTEND_GRAB = OUTTAKE_RETRACT -1584;
            OUTTAKE_ASCENT = OUTTAKE_RETRACT -754;
            OUTTAKE_EXTEND_SPECIMEN = OUTTAKE_RETRACT -550;
            OUTTAKE_EXTEND = OUTTAKE_RETRACT -1770;
            OUTTAKE_EXTEND_ASCENT_INTERMEDIARY = OUTTAKE_RETRACT -1000;

            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);

            outtakeHoming = false;
            outtakeFixSingleton = true;
        }
    }

    private void updateFollower(double power) {
        double y = -gamepad1.left_stick_y*power; // Remember, this is reversed!
        double x = gamepad1.left_stick_x*power; // this is strafing
        double rx = gamepad1.right_stick_x*power;

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

    private void fakeUpdateFollower() {
        double y = 0, x = 0, rx = 0;

        if (modeUpdateTimer.seconds() > 2) {
            if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_y != 0) {
                modeAlpha = Math.random() * 2 * Math.PI;

                y = Math.sin(modeAlpha) * modePower;
                x = Math.cos(modeAlpha) * modePower;

                rx = Math.random() * 2 - 1;
            }
            modeUpdateTimer.reset();
        }

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

    private void handleFakePress() {
        List<Object> triggers = List.of(
                gamepad1.circle, gamepad1.square, gamepad1.triangle, gamepad1.cross,
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.left_trigger, gamepad1.right_trigger,
                gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right
        );

        List<Runnable> callbacks = List.of(
                () -> {
                    robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
                    robot.intake.setPivot(UniversalValues.INTAKE_UP);
                },
                () -> {
                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                    robot.intake.setPivot(INTAKE_INT);
                },
                () ->
                    robot.outtake.ManualLevel(OUTTAKE_EXTEND, 0.3)
                ,
                () ->
                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.6)
                ,
                () -> {
                    robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                    robot.outtake.setPivot(0.9);
                    robot.intake.setPivot(INTAKE_TRANSFER);
                    robot.outtake.ManualLevel(OUTTAKE_EXTEND, 0.7);
                },
                () -> {
                    robot.intake.OpenIntake(CLAW_OPEN);
                    robot.intake.setClawPivot(CLAW_VERTICAL);
                    robot.intake.ManualLevel(INTAKE_EXTEND, 0.2);
                },
                () -> {
                    robot.outtake.ManualLevel(OUTTAKE_ASCENT, 1);
                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                    robot.outtake.ManualLevel(OUTTAKE_ASCENT, 0.7);
                    robot.outtake.setPivot(0.9);
                },
                () ->
                    modePower = Math.random() / 2
        );

        for (Object item : triggers) {
            boolean condition = false;
            if (item instanceof Boolean) {
                if ((Boolean) item) {
                    condition = true;
                }
            } else if (item instanceof Float) {
                if ((Float) item > 0.1) {
                    condition = true;
                }
            }

            if (condition) {
                int index = (int) (Math.random() * callbacks.size());
                callbacks.get(index).run();
            }
        }
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        speedTimer.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("0. SLOW MODE", toggleSpeed);
        telemetry.addData("intake  expansion", robot.intake.intakeMotor.getCurrentPosition());
        telemetry.addData("outtake expansion", robot.outtake.outtakeMotor.getCurrentPosition());
        telemetry.addData("State", intakeState);
        telemetry.addData("outtake sensor", robot.outtake.outtakeSensor.isPressed());
        telemetry.addData("intake sensor", robot.intake.intakeLimit.isPressed());

        if (speedTimer.seconds() > 0.5 && gamepad1.touchpad) {
            speedTimer.reset();
            toggleSpeed = !toggleSpeed;
        }

        if (modeTimer.seconds() > modeDelay) {
            mode = !mode;
            modeDelay = Math.random() * 14 + 1;
            modeTimer.reset();
        }

        if (mode) {
            handleFakePress();
            fakeUpdateFollower();
            telemetry.update();
            return;
        }

        outtakeHomingCheck();

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
            case ASCENT:
                handleAscent();
                break;
            default:
                intakeState = IntakeState.INTAKE_START;
                break;
        }
        if(intakeState == IntakeState.OUTTAKE_SAMPLE || toggleSpeed) {
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

    @Override
    public void start() {
        modeUpdateTimer.reset();
        modeTimer.reset();
    }
}