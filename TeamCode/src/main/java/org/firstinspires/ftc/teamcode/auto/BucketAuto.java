package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INIT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_INT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_RETRACT;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_COLLECT_NEW_TRANSFER;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_DUMP_BUCKET_DIAG;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.constants.UniversalValues.OUTTAKE_RETRACT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

@Autonomous(name = "BUCKET", group = "0. Autonomous")
public class BucketAuto extends OpMode {

    private Robot robot = null;
    private Follower follower;
    private Timer stateTimer, pathTimer, transferTimer, poseTimer;
    private Timer autoTimer;
    private boolean sliderManip = true;
    private boolean timerResetSingleton = true;

    private int stateStep = 0;

    private int pathState;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    // TODO: modify y offset to correct pedro path visualiser offset

    private final double OFFSET_X = 2.5;
    private final double OFFSET_Y = -4;

    private final Pose startPose = new Pose(9.750 + OFFSET_X, 107.000 + OFFSET_Y, Math.toRadians(-90));
    private final Pose scorePose = new Pose(15.750 + OFFSET_X, 126.250 + OFFSET_Y, Math.toRadians(-45));
    private final Pose grabFirstSamplePose = new Pose(28.700 + OFFSET_X, 128.500 + OFFSET_Y, Math.toRadians(0));
    private final Pose grabSecodSamplePose = new Pose(28.700 + OFFSET_X, 117.000 + OFFSET_Y, Math.toRadians(0));
    private final Pose grabThirdSamplePose = new Pose(32.250 + OFFSET_X, 127.000 + OFFSET_Y, Math.toRadians(35));
    private final Pose parkPose = new Pose(62.000 + OFFSET_X, 97.000 + OFFSET_Y, Math.toRadians(90));


    private PathChain scorePreload, grabFirstSample, scoreFirstSample, grabSecondSample, scoreSecondSample, grabThirdSample, scoreThirdSample, park;

    public void buildPaths() {
        // TODO : fix Linear Heading Interpolation for rotation to the right without hard coding value bigger than 180 (line 37)
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine( new Point(startPose), new Point(scorePose) )
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(), scorePose.getHeading()
                )
                .build();
        grabFirstSample = follower.pathBuilder()
                .addPath(
                        new BezierLine( new Point(scorePose), new Point(grabFirstSamplePose) )
                )
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        grabFirstSamplePose.getHeading()
                )
                .build();
        scoreFirstSample = follower.pathBuilder()
                .addPath(
                        new BezierLine( new Point(grabFirstSamplePose), new Point(scorePose) )
                )
                .setLinearHeadingInterpolation(
                        grabFirstSamplePose.getHeading(),
                        scorePose.getHeading()
                )
                .build();
        grabSecondSample = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(grabSecodSamplePose)
                        )
                )
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        grabSecodSamplePose.getHeading()
                )
                .build();
        scoreSecondSample = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabSecodSamplePose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(
                        grabSecodSamplePose.getHeading(),
                        scorePose.getHeading()
                )
                .build();
        grabThirdSample = follower.pathBuilder()
                .addPath(
                        new BezierLine( new Point(scorePose), new Point(grabThirdSamplePose) )
                )
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        grabThirdSamplePose.getHeading()
                )
                .build();
        scoreThirdSample = follower.pathBuilder()
                .addPath(
                        new BezierLine( new Point(grabThirdSamplePose), new Point(scorePose) )
                )
                .setLinearHeadingInterpolation(
                        grabThirdSamplePose.getHeading(),
                        scorePose.getHeading()
                )
                .build();
        park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(60.000 + OFFSET_X, 128.000 + OFFSET_Y, Point.CARTESIAN),
                                new Point(parkPose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;

        robot.universalTransfer.resetTransfer();
        pathTimer.resetTimer();
        stateTimer.resetTimer();
        transferTimer.resetTimer();
        sliderManip = true;
        timerResetSingleton = true;

        stateStep = 0;
    }

    public void autonomousUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(scorePreload,true);

                setPathState(1);
                break;
            case 1:
                if ((follower.getPose().getX() > scorePose.getX() - 1) &&
                        (follower.getPose().getY() > scorePose.getY() - 1)) {
                    if (timerResetSingleton) {
                        poseTimer.resetTimer();
                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0.5 && stateStep == 0) {
                        robot.outtake.ManualLevel(OUTTAKE_EXTEND, 1);
                        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.5 && stateStep == 1) {
                        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET_DIAG);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 2.1 && stateStep == 2) {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        ++stateStep;
                    }
                    if (poseTimer.getElapsedTimeSeconds() > 2.35 && stateStep == 3) {
                        follower.followPath(grabFirstSample, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if ((follower.getPose().getX() > grabFirstSamplePose.getX() - 1) &&
                        (follower.getPose().getY() < grabFirstSamplePose.getY() + 1)) {
                    if (timerResetSingleton) {
                        poseTimer.resetTimer();

                        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                        robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
                        robot.intake.OpenIntake(CLAW_OPEN);

                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0.5 && stateStep == 0) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        sliderManip = false;
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0.7 && stateStep == 1) {
                        robot.intake.setPivot(INTAKE_DOWN);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.1 && stateStep == 2) {
                        robot.intake.CloseIntake(CLAW_CLOSE);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.4 && stateStep == 3) {
                        robot.universalTransfer.transfer();
                        if (robot.universalTransfer.isTransferCompleted()) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND, 0.8);
                            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                            follower.followPath(scoreFirstSample, true);
                            setPathState(3);
                        }

                    }
                }
                break;
            case 3:
                if ((follower.getPose().getX() < scorePose.getX() + 1) &&
                        (follower.getPose().getY() < scorePose.getY() + 1)) {


                    if (timerResetSingleton) {
                        poseTimer.resetTimer();
                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1 && stateStep == 0) {
                        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET_DIAG);

                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.4 && stateStep == 1) {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.6 && stateStep == 2) {
                        robot.intake.setPivot(INTAKE_INT);

                        follower.followPath(grabSecondSample, true);
                        robot.universalTransfer.resetTransfer();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if ((follower.getPose().getX() > grabSecodSamplePose.getX() - 1) &&
                        (follower.getPose().getY() < grabSecodSamplePose.getY() + 1)) {

                    if (timerResetSingleton) {
                        poseTimer.resetTimer();

                        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                        robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
                        robot.intake.OpenIntake(CLAW_OPEN);

                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0. && stateStep == 0) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        sliderManip = false;
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0.7 && stateStep == 1) {
                        robot.intake.setPivot(INTAKE_DOWN);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.1 && stateStep == 2) {
                        robot.intake.CloseIntake(CLAW_CLOSE);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.4 && stateStep == 3) {
                        robot.universalTransfer.transfer();
                        if (robot.universalTransfer.isTransferCompleted()) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND, 0.8);
                            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                            follower.followPath(scoreSecondSample, true);
                            setPathState(5);
                        }
                    }
                }
                break;

            case 5:
                if ((follower.getPose().getX() < scorePose.getX() + 1) &&
                        (follower.getPose().getY() > scorePose.getY() - 1)) {

                    if (timerResetSingleton) {
                        poseTimer.resetTimer();
                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1 && stateStep == 0) {
                        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET_DIAG);

                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.4 && stateStep == 1) {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.6 && stateStep == 2) {
                        robot.intake.setPivot(INTAKE_INT);

                        follower.followPath(grabThirdSample, true);
                        robot.universalTransfer.resetTransfer();
                        setPathState(6);
                    }

                }
                break;
            case 6:
                if ((follower.getPose().getX() > grabThirdSamplePose.getX() - 1) &&
                        (follower.getPose().getY() > grabThirdSamplePose.getY() - 1)) {
                    if (timerResetSingleton) {
                        poseTimer.resetTimer();

                        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                        robot.intake.setClawPivot(0.56);
                        robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
                        robot.intake.OpenIntake(CLAW_OPEN);

                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0.5 && stateStep == 0) {
                        robot.intake.setPivot(INTAKE_DOWN);

                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 0.7 && stateStep == 1) {
                        robot.intake.ManualLevel(INTAKE_EXTEND, 0.8);
                        sliderManip = false;

                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.1 && stateStep == 2) {
                        robot.intake.CloseIntake(CLAW_CLOSE);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.4 && stateStep == 3) {
                        robot.intake.ManualLevel(INTAKE_RETRACT, 0.8);
                        robot.intake.setPivot(INTAKE_UP);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.7 && stateStep == 4) {
                        robot.universalTransfer.transfer();
                        if (robot.universalTransfer.isTransferCompleted()) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND, 0.8);
                            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                            follower.followPath(scoreThirdSample, true);
                            setPathState(7);
                        }
                    }
                }
                break;

            case (7):

                if ((follower.getPose().getX() < scorePose.getX() + 1) &&
                        (follower.getPose().getY() > scorePose.getY() - 1)) {

                    if (timerResetSingleton) {
                        poseTimer.resetTimer();
                        timerResetSingleton = false;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1 && stateStep == 0) {
                        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET_DIAG);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.4 && stateStep == 1) {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        ++stateStep;
                    }

                    if (poseTimer.getElapsedTimeSeconds() > 1.6 && stateStep == 2) {

                        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);

                        follower.followPath(park, true);
                        robot.universalTransfer.resetTransfer();
                        setPathState(8);
                    }
                }
                break;


            case 8:
                robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.8);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                       robot.outtake.setPivot(0.8);
                }

                if ((follower.getPose().getX() > parkPose.getX() - 1) &&
                        (follower.getPose().getY() < parkPose.getY() + 1)) {
                    setPathState(-1);
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
        poseTimer = new Timer();

        follower.setStartingPose(startPose);

        robot.intake.ManualLevel(INTAKE_RETRACT, 1);
        robot.intake.CloseIntake(CLAW_OPEN);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT);
        robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
        robot.outtake.CloseOuttake(OUTTAKE_CLOSE);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

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

        autoTimer = new Timer();
    }

    @Override
    public void loop() {

        follower.update();
        if (sliderManip) {
            robot.intake.ManualLevel(INTAKE_RETRACT, 1);
        }
        autonomousUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("timer", autoTimer.getElapsedTimeSeconds());
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
        autoTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}