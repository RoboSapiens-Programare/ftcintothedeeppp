package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_TIMER;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_VERTICAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_CLIPON_DOWN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_CLIPON_UP;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_COLLECT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_DUMP_BUCKET;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND_MID;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.PIVOT_TIMER;
import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/* THIS IS EXACTLY THE CODE FROM FSM DRIVE MODE
 *  It simply adds an autonomous path when DRIVER #1 presses CIRCLE after PICKING UP the sample
 *  Pressing it once again should override it and stop the path following.
 *  Once that happened, the path won't be followed from where it stopped, but from the beginning.
 *  THIS MODEL ONLY ACCOUNTS FOR TAKING SAMPLES FROM THE SPECIMEN-SIDE IN THE TRACTION ZONE
 *  Ex start point for Pedro Path generator: 70, 40 */

// CURRENT STATUS: NOT TESTED

@Disabled
@TeleOp(name = "SMART FSM DRIVE MODE", group = "FSMTELEOP")
public class smartFSMDriveMode extends OpMode {

    private double clawPivot = CLAW_HORIZONTAL;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();


    // TODO: change to last pose of the autonomy
    private final Pose startPose = new Pose(2, 32, 0);
    private final Pose pickup = new Pose(72, 47, Math.toRadians(90));
    private final Pose observationZone = new Pose(22, 12, Math.toRadians(180));
    private final Pose observationZoneWait = new Pose(30, 12, Math.toRadians(180));
    private PathChain toObservationZonePath, retreat;

    private Robot robot;
    private Follower follower;
    private boolean isHorizontal = true;
    private boolean isStarted = true;
    private boolean isPressed = false;
    private boolean isSquare = false;
    private boolean isTimer = true;
    private boolean isMoving = false;
    private boolean isAutoRunning = false;
    private boolean autoSingleton = true;
    private Timer actionTimer;
    private enum IntakeState {
        INTAKE_START, INTAKE_CLAW_COLLECT_POSITION, INTAKE_RETRACT, INTAKE_EXTEND,
        OUTTAKE_MID, OUTTAKE_EXTEND, OUTTAKE_RETRACT, OUTTAKE_SAMPLE,

        AUTO
    }

    private enum AutoState {
        DEFAULT, COLLECT, TO_OBSERVATION_ZONE, RETREAT
    }
    private AutoState autoState = AutoState.COLLECT;
    private IntakeState intakeState = IntakeState.INTAKE_START;

    private void generatePaths() {
        toObservationZonePath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup),
                                new Point(75.893, 10.809, Point.CARTESIAN),
                                new Point(observationZone)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        retreat = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(observationZone), new Point(observationZoneWait))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void initializeRobot() {
        robot.intake.ManualLevel(0, 1);
        robot.intake.CloseIntake(CLAW_CLOSE);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT);
        robot.outtake.setPivot(OUTTAKE_COLLECT);
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
            robot.outtake.setPivot(OUTTAKE_COLLECT);
        } else if (gamepad1.right_bumper) {
            robot.intake.OpenIntake(CLAW_OPEN - 0.185);
            robot.outtake.setPivot(OUTTAKE_COLLECT);
        }

        adjustClawPivot();
        adjustClawPosition();

        if (gamepad1.circle) {
            isAutoRunning = true;
            intakeState = IntakeState.AUTO;
            return;
        }

        if (gamepad1.left_trigger > 0.1) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            robot.intake.setPivot(INTAKE_UP);
            if (intakeTimer.seconds() > 0.5) {
                robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                intakeTimer.reset();
                intakeState = IntakeState.INTAKE_RETRACT;
            }
        }
        if (gamepad1.cross) {
            robot.intake.setPivot(INTAKE_INT);
        }
        if (gamepad1.square) {
            robot.intake.setPivot(INTAKE_DOWN);
        }
    }

    private void handleIntakeRetract() {
        if (robot.intake.intakeLimit.isPressed()) isPressed = true;
        if (isPressed) {
            robot.intake.setPivot(INTAKE_UP);
            if (intakeTimer.seconds() > CLAW_TIMER) {
                robot.intake.OpenIntake(CLAW_OPEN);
                robot.outtake.CloseOuttake(OUTTAKE_CLOSE - 0.07);
                if (intakeTimer.seconds() > CLAW_TIMER + 0.3) {
                    robot.intake.ManualLevel(300, 1);
                    intakeTimer.reset();
                }
            }
            if (robot.intake.intakeMotor.getCurrentPosition() > 200) {
                robot.intake.setPivot(INTAKE_INT);
                robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                intakeState = IntakeState.INTAKE_START;
            }
        }
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
        robot.outtake.setPivot(OUTTAKE_COLLECT);
        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        intakeState = IntakeState.INTAKE_START;
    }

    private void handleOuttakeSample() {
        if (gamepad1.square && !isSquare) {
            isSquare = true;
            isTimer = true;
        }
        if (abs(gamepad1.left_stick_y) > 0.02 && isSquare && !isMoving) {
            isMoving = true;
        }
        if (isSquare && isMoving) {
            if (isTimer) {
                intakeTimer.reset();
                isTimer = false;
            }
            robot.outtake.setPivot(OUTTAKE_CLIPON_DOWN);
            if (intakeTimer.seconds() > 0.3) {
                robot.outtake.OpenOuttake(OUTTAKE_OPEN);

                if (intakeTimer.seconds() > 0.5) {
                    robot.outtake.setPivot(OUTTAKE_CLIPON_UP);
                    robot.outtake.setPivot(OUTTAKE_COLLECT);
                    intakeState = IntakeState.INTAKE_START;
                    intakeTimer.reset();
                }
            }
        }
    }

    private void changeAutoState(AutoState state) {
        autoState = state;
        actionTimer.resetTimer();
        autoSingleton = true;
    }

    private void handleAuto() {
        // STOP AUTO
        if (gamepad1.circle) {
            isAutoRunning = false;
            changeAutoState(AutoState.COLLECT);
            intakeState = IntakeState.INTAKE_CLAW_COLLECT_POSITION;
            return;
        }

        switch (autoState) {
            // After sample has been grabbed, retract the claw
            case COLLECT:
                robot.intake.setPivot(INTAKE_INT);
                robot.intake.setClawPivot(CLAW_HORIZONTAL);
                clawPivot = CLAW_HORIZONTAL;
                robot.intake.ManualLevel(INTAKE_RETRACT, 1);

                changeAutoState(AutoState.TO_OBSERVATION_ZONE);
                break;

            // After claw has retracted,
            case TO_OBSERVATION_ZONE:
                if (autoSingleton && actionTimer.getElapsedTimeSeconds() > 3) {
                    robot.intake.setPivot(INTAKE_UP);
                    autoSingleton = false;
                }

                follower.followPath(toObservationZonePath);

                changeAutoState(AutoState.RETREAT);
                break;
            case RETREAT:
                // got to observation zone position
                if ((follower.getPose().getX() < observationZone.getX() + 1) && (follower.getPose().getY() < observationZone.getY() + 1)) {
                    if (autoSingleton) {
                        robot.intake.setPivot(INTAKE_INT);
                        robot.intake.OpenIntake(CLAW_OPEN);
                        actionTimer.resetTimer();
                        autoSingleton = false;
                    }
                    // if there and had let down the sample for 1 sec, retreat
                    if (actionTimer.getElapsedTimeSeconds() > 1 && !autoSingleton) {
                        follower.followPath(retreat);
                        changeAutoState(AutoState.DEFAULT);
                    }
                }
                break;
            default:
                break;
        }
    }

    private void updateFollower(double power) {
        if (!isAutoRunning) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.setMaxPower(power);
        }
        follower.update();
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);
        generatePaths();

        intakeTimer.reset();
        outtakeTimer.reset();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        follower.startTeleopDrive();
        initializeRobot();
    }

    @Override
    public void loop() {
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
            case INTAKE_RETRACT:
                handleIntakeRetract();
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

            case AUTO:
                handleAuto();
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
                break;
        }
        if (intakeState == IntakeState.OUTTAKE_SAMPLE) {
            updateFollower(0.4);
        } else updateFollower(1);
        telemetry.update();
    }
}