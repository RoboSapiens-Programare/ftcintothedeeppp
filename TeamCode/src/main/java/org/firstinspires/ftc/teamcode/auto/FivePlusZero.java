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

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;

@Autonomous(name = "5+0", group = "0. Autonomous")
public class FivePlusZero extends OpMode {

    // TODO fine tune positions and add claw pivot for last pickup

    private Robot robot = null;
    private Follower follower;
    private Timer stateTimer, pathTimer, transferTimer;
    private boolean singleton = true;
    private boolean singleton2 = true;
    private boolean singleton3 = true;
    private boolean singleton4 = true;
    private int pathState;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private final Pose startPose = new Pose(8.5,44, Math.toRadians(180));

    private final Pose barCliponPose1 = new Pose(35,60.5, Math.toRadians(180));
    private final Pose barCliponPose2 = new Pose(35,63.5, Math.toRadians(180));
    private final Pose barCliponPose3 = new Pose(35, 65.5, Math.toRadians(180));
    private final Pose barCliponPose4 = new Pose(35, 67.5, Math.toRadians(180));
    private final Pose barCliponPose5 = new Pose(35, 69.5, Math.toRadians(180));

    private final Pose samplePickup1 = new Pose(32.7, 22);
    private final Pose sampleDropOff1 = new Pose(32.7, 20);
    private final Pose samplePickup2 = new Pose(32.7, 12);
    private final Pose sampleDropOff2 = new Pose(32.7, 10);
    private final Pose samplePickup3 = new Pose(32.7, 9);

    private final Pose specimenPickup = new Pose(21.1, 24, Math.toRadians(180));

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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                    .build();

            line3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(samplePickup1),
                                    new Point(sampleDropOff1)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                    .build();

            line4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(sampleDropOff1),
                                    new Point(samplePickup2)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                    .build();

            line5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(samplePickup2),
                                    new Point(sampleDropOff2)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                    .build();

            line6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(sampleDropOff2),
                                    new Point(samplePickup3)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-25))
                    .build();

            line7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(samplePickup3),
                                    new Point(specimenPickup)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(180))
                    .build();

            line8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(specimenPickup),
                                    new Point(barCliponPose2)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(barCliponPose2),
                                    new Point(specimenPickup)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line10 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(specimenPickup),
                                    new Point(barCliponPose3)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(barCliponPose3),
                                    new Point(specimenPickup)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line12 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(specimenPickup),
                                    new Point(barCliponPose4)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line13 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(barCliponPose4),
                                    new Point(specimenPickup)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line14 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(specimenPickup),
                                    new Point(barCliponPose5)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line15 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(barCliponPose5),
                                    new Point(ParkPose)
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
        singleton = true;
        singleton2 = true;
        singleton3 = true;
        singleton4 = true;
    }

    public void autonomousUpdate() {
        switch(pathState) {
            case(0):
                if (singleton)
                {
                    follower.followPath(line1,true);
                    robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
                    singleton = false;
                }
                if (!follower.isBusy()) {
                    if (singleton2)
                    {
                        stateTimer.resetTimer();
                        singleton2 = false;
                    }
                    robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                    if (stateTimer.getElapsedTimeSeconds()>0.3)
                    {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        if (stateTimer.getElapsedTimeSeconds()>0.6)
                        {
                            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                            setPathState(1);
                        }
                    }
                }
                break;
            case(1):
                if (singleton2)
                {
                    follower.followPath(line2,true);
                    robot.intake.OpenIntake(CLAW_OPEN);
                    robot.intake.setPivot(INTAKE_DOWN);
                    robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                    singleton2 = false;
                }

                if (!follower.isBusy() && singleton)
                {
                    stateTimer.resetTimer();
                    singleton = false;
                }

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>2) {
                    setPathState(2);
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>1) {
                    robot.intake.setPivot(INTAKE_INT);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>0.5) {
                    robot.intake.OpenIntake(CLAW_CLOSE);
                }
                break;
            case(2):
                if (singleton3)
                {
                    follower.followPath(line3,true);
                    singleton3 = false;
                }

                if (!follower.isBusy() && singleton)
                {
                    stateTimer.resetTimer();
                    singleton = false;
                }

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>2) {
                    setPathState(3);
                    robot.intake.setPivot(INTAKE_INT);
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>1) {
                    robot.intake.OpenIntake(CLAW_OPEN);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>0.5) {
                    robot.intake.ManualLevel(INTAKE_EXTEND, 1);
                    robot.intake.setPivot(INTAKE_DOWN);
                }
                break;
            case(3):
                if (singleton2)
                {
                    follower.followPath(line4,true);
                    singleton2 = false;
                }

                if (!follower.isBusy() && singleton)
                {
                    stateTimer.resetTimer();
                    singleton = false;
                    robot.intake.ManualLevel(INTAKE_EXTEND, 1);
                    robot.intake.setPivot(INTAKE_DOWN);
                }

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>2) {
                    setPathState(4);
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>1) {
                    robot.intake.setPivot(INTAKE_INT);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>0.5) {
                    robot.intake.OpenIntake(CLAW_CLOSE);
                }
                break;
            case(4):
                if (singleton2)
                {
                    follower.followPath(line5,true);
                    singleton2 = false;
                }

                if (!follower.isBusy() && singleton)
                {
                    stateTimer.resetTimer();
                    singleton = false;
                }

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>2) {
                    setPathState(5);
                    robot.intake.setPivot(INTAKE_INT);
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>1) {
                    robot.intake.OpenIntake(CLAW_OPEN);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>0.5) {
                    robot.intake.ManualLevel(INTAKE_EXTEND, 1);
                    robot.intake.setPivot(INTAKE_DOWN);
                }
                break;
            case(5):
                if (singleton3)
                {
                    follower.followPath(line6,true);
                    singleton3 = false;
                    robot.intake.setPivot(INTAKE_DOWN);
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                }

                if (!follower.isBusy() && singleton)
                {
                    stateTimer.resetTimer();
                    singleton = false;
                }

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>2) {
                    setPathState(6);
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>1) {
                    robot.intake.OpenIntake(CLAW_CLOSE);
                } else if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>0.5) {
                    robot.intake.ManualLevel(INTAKE_EXTEND, 1);

                }
                break;
            case(6):
                if (singleton2)
                {
                    follower.followPath(line7,true);
                    singleton2 = false;
                }

                if (!follower.isBusy() && singleton4)
                {
                    stateTimer.resetTimer();
                    robot.intake.ManualLevel(INTAKE_EXTEND/2, 1);
                    singleton4 = false;
                }

                if (stateTimer.getElapsedTimeSeconds()>1.5 && stateTimer.getElapsedTimeSeconds()<1.51 && singleton)
                {
                    robot.intake.setPivot(INTAKE_DOWN);
                    robot.intake.OpenIntake(CLAW_OPEN);
                } else if (stateTimer.getElapsedTimeSeconds()>2.5 && stateTimer.getElapsedTimeSeconds()<2.51 && singleton) {
                    robot.intake.ManualLevel(INTAKE_RETRACT, 1);
                    robot.intake.setPivot(UniversalValues.INTAKE_INT);
                }


                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds()>3 || !singleton) {

                    if (singleton)
                    {
                        stateTimer.resetTimer();
                        robot.intake.setPivot(UniversalValues.INTAKE_INT);
                        robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                        robot.intake.ManualLevel(INTAKE_RETRACT+150, 1);
                        singleton = false;
                    }

                    if (stateTimer.getElapsedTimeSeconds() > 2.1)
                    {
                        robot.intake.OpenIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 2.3) {

                            follower.followPath(line7, true);
                            setPathState(7);
                            // starts cycling
                        }
                    }
                }
                break;


            case(7):
                robot.universalTransfer.transfer();
                if (singleton2)
                {
                    follower.followPath(line8,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton4)
                        {
                            stateTimer.resetTimer();
                            singleton4 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                                    robot.universalTransfer.resetTransfer();
                                    setPathState(8);
                                }
                            }
                        }
                    }
                }
                break;
            case(8):
                if (singleton2)
                {
                    follower.followPath(line9,true);
                    singleton2 = false;
                }

                if (!follower.isBusy() || !singleton) {

                    if (singleton3)
                    {
                        stateTimer.resetTimer();
                        robot.intake.setPivot(UniversalValues.INTAKE_INT);
                        robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                        singleton3 = false;
                    }



                    if (singleton)
                    {
                        stateTimer.resetTimer();
                        singleton = false;
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 1.7)
                    {
                        singleton3 = false;
                        robot.intake.ManualLevel(INTAKE_RETRACT+150, 1);
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 2.1)
                    {
                        robot.intake.OpenIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 2.3) {

                            setPathState(9);
                        }
                    }
                }
                break;
            case(9):
                robot.universalTransfer.transfer();
                if (singleton2)
                {
                    follower.followPath(line10,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton4)
                        {
                            stateTimer.resetTimer();
                            singleton4 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                                    robot.universalTransfer.resetTransfer();
                                    setPathState(10);
                                }
                            }
                        }
                    }
                }
                break;
            case(10):
                if (singleton2)
                {
                    follower.followPath(line11,true);
                    singleton2 = false;
                }

                if (!follower.isBusy() || !singleton) {

                    if (singleton3)
                    {
                        robot.intake.setPivot(UniversalValues.INTAKE_INT);
                        robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                        stateTimer.resetTimer();
                        singleton3 = false;
                    }



                    if (singleton)
                    {
                        stateTimer.resetTimer();
                        singleton = false;
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 1.7)
                    {
                        singleton3 = false;
                        robot.intake.ManualLevel(INTAKE_RETRACT+150, 1);
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 2.1)
                    {
                        robot.intake.OpenIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 2.3) {

                            setPathState(11);
                        }
                    }
                }
                break;
            case(11):
                robot.universalTransfer.transfer();
                if (singleton2)
                {
                    follower.followPath(line12,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton4)
                        {
                            stateTimer.resetTimer();
                            singleton4 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                                    robot.universalTransfer.resetTransfer();
                                    setPathState(12);
                                }
                            }
                        }
                    }
                }
                break;
            case(12):
                if (singleton2)
                {
                    follower.followPath(line13,true);
                    singleton2 = false;
                }

                if (!follower.isBusy() || !singleton) {

                    if (singleton3)
                    {
                        stateTimer.resetTimer();
                        singleton3 = false;
                    }

                    if (singleton)
                    {
                        stateTimer.resetTimer();
                        singleton = false;
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 1.7)
                    {
                        singleton3 = false;
                        robot.intake.ManualLevel(INTAKE_RETRACT+150, 1);
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 2.1)
                    {
                        robot.intake.OpenIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 2.3) {

                            setPathState(13);
                        }
                    }
                }
                break;
            case(13):
                robot.universalTransfer.transfer();
                if (singleton2)
                {
                    follower.followPath(line14,true);
                    singleton2 = false;
                }
                if (!follower.isBusy()) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton4)
                        {
                            stateTimer.resetTimer();
                            singleton4 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                                    robot.universalTransfer.resetTransfer();
                                    setPathState(14);
                                }
                            }
                        }
                    }
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
        follower = new Follower(hardwareMap);

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
//        if (singleton3) {
//            robot.intake.ManualLevel(INTAKE_RETRACT, 1);
//        }
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