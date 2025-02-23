package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INIT;
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

@Autonomous(name = "5+0", group = "0. Autonomous")
public class FivePlusZero extends OpMode {

    private Robot robot = null;
    private Follower follower;
    private Timer stateTimer, pathTimer, transferTimer;
    private boolean singleton = true;
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

    private final Pose samplePickup = new Pose(27.5, 13.1);

    private final Pose specimenPickup = new Pose(20.1, 24, Math.toRadians(180));

    private final Pose ParkPose = new Pose(15,27, Math.toRadians(180));


    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12, line13, line14, line15, line16;

    public void buildPaths() {

            line1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(8.536, 44.000, Point.CARTESIAN),
                                    new Point(35.750, 62.500, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(35.750, 62.500, Point.CARTESIAN),
                                    new Point(27.500, 13.100, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            line3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(27.500, 13.100, Point.CARTESIAN),
                                    new Point(27.500, 13.100, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            line4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(27.500, 13.100, Point.CARTESIAN),
                                    new Point(27.500, 13.100, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                    .build();

            line5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(27.500, 13.100, Point.CARTESIAN),
                                    new Point(27.500, 13.100, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                    .build();

            line6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(27.500, 13.100, Point.CARTESIAN),
                                    new Point(27.500, 13.100, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            line7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(27.500, 13.100, Point.CARTESIAN),
                                    new Point(27.500, 13.100, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                    .build();

            line8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(27.500, 13.100, Point.CARTESIAN),
                                    new Point(20.100, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(20.100, 24.000, Point.CARTESIAN),
                                    new Point(36.500, 69.500, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line10 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(36.500, 69.500, Point.CARTESIAN),
                                    new Point(20.100, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(20.100, 24.000, Point.CARTESIAN),
                                    new Point(36.500, 73.500, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line12 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(36.500, 73.500, Point.CARTESIAN),
                                    new Point(20.100, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line13 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(20.100, 24.000, Point.CARTESIAN),
                                    new Point(36.700, 78.500, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line14 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(36.700, 78.500, Point.CARTESIAN),
                                    new Point(20.100, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line15 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(20.100, 24.000, Point.CARTESIAN),
                                    new Point(36.700, 80.500, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            line16 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(36.700, 80.500, Point.CARTESIAN),
                                    new Point(15.000, 27.000, Point.CARTESIAN)
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
    }

    public void autonomousUpdate() {
        switch(pathState) {
            case(0):
                follower.followPath(line1,true);
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;
            case(1):
                follower.followPath(line2,true);
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case(2):
                follower.followPath(line3,true);
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;
            case(3):
                follower.followPath(line4,true);
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case(4):
                follower.followPath(line5,true);
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case(5):
                follower.followPath(line6,true);
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case(6):
                follower.followPath(line7,true);
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case(7):
                follower.followPath(line8,true);
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case(8):
                follower.followPath(line9,true);
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case(9):
                follower.followPath(line10,true);
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case(10):
                follower.followPath(line11,true);
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case(11):
                follower.followPath(line12,true);
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case(12):
                follower.followPath(line13,true);
                if (!follower.isBusy()) {
                    setPathState(13);
                }
                break;
            case(13):
                follower.followPath(line14,true);
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case(14):
                follower.followPath(line15,true);
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;
            case(15):
                follower.followPath(line16,true);
                if (!follower.isBusy()) {
                    setPathState(16);
                }
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
        if (singleton3) {
            robot.intake.ManualLevel(INTAKE_RETRACT, 1);
        }
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