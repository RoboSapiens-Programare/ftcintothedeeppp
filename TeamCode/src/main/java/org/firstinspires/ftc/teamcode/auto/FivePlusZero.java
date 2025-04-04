package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_DUMP_BUCKET;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND_SPECIMEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_RETRACT;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.constants.UniversalValues;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

@Autonomous(name = "5+0", group = "0. Autonomous")
public class FivePlusZero extends OpMode {

    private Robot robot = null;
    private Follower follower;
    private Timer stateTimer, pathTimer, transferTimer;
    private boolean singleton = true;
    private boolean stateTimerResetSingleton = true;
    private boolean singleton2 = true;
    private boolean singleton3 = true;
    private int pathState;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    // TODO: modify y offset to correct pedro path visualiser offset (currently about -5)


    private final Pose startPose = new Pose(8.5,44, Math.toRadians(180));

    private final Pose barCliponPose1 = new Pose(35.75,62.5, Math.toRadians(180));
    private final Pose barCliponPose2 = new Pose(36.5,69.5, Math.toRadians(180));
    private final Pose barCliponPose3 = new Pose(36.5, 73.5, Math.toRadians(180));
    private final Pose barCliponPose4 = new Pose(36.7, 78.5, Math.toRadians(180));
    private final Pose barCliponPose5 = new Pose(36.7, 80.5, Math.toRadians(180));

    private final Pose samplePickup1 = new Pose(32.7, 28.2);
    private final Pose sampleDropOff1 = new Pose(32.7, 26.7);
    private final Pose samplePickup2 = new Pose(32.7, 24.2);
    private final Pose sampleDropOff2 = new Pose(32.7, 20.7);
    private final Pose samplePickup3 = new Pose(32.7, 16.2);
    private final Pose sampleDropOff3 = new Pose(32.7, 21.7);


    private final Pose specimenPickup = new Pose(18.1, 47, Math.toRadians(225));

    private final Pose ParkPose = new Pose(15,27, Math.toRadians(180));


    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15;

    public void buildPaths() {

        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(barCliponPose1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(barCliponPose1),
                                new Point(samplePickup1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(samplePickup1),
                                new Point(sampleDropOff1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-170))
                .build();

        line4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(sampleDropOff1),
                                new Point(samplePickup2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(-45))
                .build();

        line5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(samplePickup2),
                                new Point(sampleDropOff2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-170))
                .build();

        line6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(sampleDropOff2),
                                new Point(samplePickup3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(-45))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(samplePickup3),
                                new Point(sampleDropOff3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                .build();

        line8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(sampleDropOff3),
                                new Point(specimenPickup)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenPickup),
                                new Point(barCliponPose2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(barCliponPose2),
                                new Point(specimenPickup)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenPickup),
                                new Point(barCliponPose3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(barCliponPose3),
                                new Point(specimenPickup)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenPickup),
                                new Point(barCliponPose4)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line14 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(barCliponPose4),
                                new Point(specimenPickup)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        line15 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenPickup),
                                new Point(barCliponPose5)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    public void setPathState(int pState) {
        pathState = pState;

        robot.universalTransfer.resetTransfer();
        pathTimer.resetTimer();
        stateTimer.resetTimer();
        transferTimer.resetTimer();
        stateTimerResetSingleton = true;
        singleton = true;
        singleton2 = true;
        singleton3 = true;
    }

    private void resetState() {
        if (stateTimerResetSingleton) {
            stateTimer.resetTimer();
            stateTimerResetSingleton = false;
        }
    }

    public void autonomousUpdate() {
        switch(pathState) {
            case(0):
                if (singleton) {
                    robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);

                    follower.followPath(line1,true);
                    singleton = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 0.8);
                    if (stateTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        robot.intake.setPivot(INTAKE_INT);
                        setPathState(1);
                    }
                }
                break;
            case(1):
                if (singleton2) {
                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);

                    follower.followPath(line2,true);
                    singleton2 = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    if (singleton) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        robot.intake.setClawPivot(0.5);
                        robot.intake.setPivot(INTAKE_DOWN);

                        singleton = false;
                    }

                    if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.intake.CloseIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                            robot.intake.setPivot(INTAKE_INT);
                            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                            robot.intake.setClawPivot(CLAW_HORIZONTAL);

                            setPathState(2);
                        }
                    }

                }
                break;
            case(2):
                if (singleton2) {
                    follower.followPath(line3,true);
                    singleton2 = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    if (singleton) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        singleton = false;
                    }
                    robot.intake.setPivot(INTAKE_DOWN);

                    if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.intake.OpenIntake(CLAW_OPEN);
                        if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                            robot.intake.setPivot(INTAKE_INT);
                            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                            setPathState(3);
                        }
                    }
                }
                break;
            case(3):
                if (singleton2) {
                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);

                    follower.followPath(line4,true);
                    singleton2 = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    if (singleton) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        robot.intake.setClawPivot(0.5);
                        robot.intake.setPivot(INTAKE_DOWN);

                        singleton = false;
                    }

                    if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.intake.CloseIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                            robot.intake.setPivot(INTAKE_INT);
                            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                            robot.intake.setClawPivot(CLAW_HORIZONTAL);

                            setPathState(4);
                        }
                    }

                }
                break;
            case(4):
                if (singleton2) {
                    follower.followPath(line5,true);
                    singleton2 = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    if (singleton) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        singleton = false;
                    }
                    robot.intake.setPivot(INTAKE_DOWN);

                    if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.intake.OpenIntake(CLAW_OPEN);
                        if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                            robot.intake.setPivot(INTAKE_INT);
                            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                            setPathState(5);
                        }
                    }
                }
                break;
            case(5):
                if (singleton2) {
                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);

                    follower.followPath(line6,true);
                    singleton2 = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    if (singleton) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        robot.intake.setClawPivot(0.5);
                        robot.intake.setPivot(INTAKE_DOWN);

                        singleton = false;
                    }

                    if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.intake.CloseIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                            robot.intake.setPivot(INTAKE_INT);
                            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                            robot.intake.setClawPivot(CLAW_HORIZONTAL);

                            setPathState(6);
                        }
                    }

                }
                break;
            case(6):
                if (singleton2) {
                    follower.followPath(line7,true);
                    singleton2 = false;
                }

                if (!follower.isBusy()) {
                    resetState();

                    if (singleton) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        singleton = false;
                    }
                    robot.intake.setPivot(INTAKE_DOWN);

                    if (stateTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.intake.OpenIntake(CLAW_OPEN);
                        if (stateTimer.getElapsedTimeSeconds() > 0.4) {
                            robot.intake.setPivot(INTAKE_INT);
                            robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                            setPathState(7);
                        }
                    }
                }
                break;
            case(7):
                if (singleton2)
                {
                    follower.followPath(line8,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    resetState();
                    setPathState(8);
                }
                break;
            case(8):
                if (singleton3)
                {
                    follower.followPath(line9,true);
                    singleton3 = false;
                }
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case(9):
                if (singleton)
                {
                    follower.followPath(line10,true);
                    singleton = false;
                }
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case(10):
                if (singleton2)
                {
                    follower.followPath(line11,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case(11):
                if (singleton3)
                {
                    follower.followPath(line12,true);
                    singleton3 = false;
                }
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case(12):
                if (singleton)
                {
                    follower.followPath(line13,true);
                    singleton = false;
                }
                if (!follower.isBusy()) {
                    setPathState(13);
                }
                break;
            case(13):
                if (singleton2)
                {
                    follower.followPath(line14,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case(14):
                if (singleton3)
                {
                    follower.followPath(line15,true);
                    singleton3 = false;
                }
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;
            case(15):
                setPathState(-1);
                break;
        }
    }


    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);

        robot = new Robot(hardwareMap);
        follower = new Follower(hardwareMap, LConstants.class, FConstants.class);

        stateTimer = new Timer();
        pathTimer = new Timer();
        transferTimer = new Timer();

        follower.setStartingPose(startPose);

        robot.intake.ManualLevel(INTAKE_RETRACT, 1);
        robot.intake.CloseIntake(CLAW_OPEN);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT-0.1);
        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(OUTTAKE_OPEN);

        telemetry.update();
        buildPaths();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {

        follower.update();
        autonomousUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }


    @Override
    public void init_loop() {
        stateTimer.resetTimer();
        transferTimer.resetTimer();

        robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
        robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
    }

    @Override
    public void start() {
        setPathState(0);
        robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
        robot.intake.setPivot(INTAKE_INIT);
    }

    @Override
    public void stop() {
    }
}