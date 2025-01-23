package org.firstinspires.ftc.teamcode.auto;

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

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.universalValues;

@Autonomous(name="Bucket Auto New", group = "Autonomous")
public class BucketAutoNew extends OpMode {
    private org.firstinspires.ftc.teamcode.subsystems.robot robot = null;
    private Follower follower;
    private Timer actionTimer;
    private int step = 0;

    private PathChain scorePreload, grabFirstSample, scoreFirstSample, grabSecondSample, scoreSecondSample, pushThirdSample, park;

    private final double OFFSET_X = 0;
    private final double OFFSET_Y = 0;

    private final Pose startPose = new Pose(9.750 + OFFSET_X, 85.000 + OFFSET_Y, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16.200 + OFFSET_X, 128.000 + OFFSET_Y, Math.toRadians(-45));
    private final Pose scorePushPose = new Pose(14.000 + OFFSET_X, 135.000 + OFFSET_Y, Math.toRadians(-90));
    private final Pose grabFirstSamplePose = new Pose(28.000 + OFFSET_X, 131.500 + OFFSET_Y, Math.toRadians(0));
    private final Pose grabSecodSamplePose = new Pose(28.000 + OFFSET_X, 121.500 + OFFSET_Y, Math.toRadians(0));
    private final Pose parkPose = new Pose(62.000 + OFFSET_X, 95.000 + OFFSET_Y, Math.toRadians(-90));

    private enum PathState {
        START,
        SCORE_PRELOAD,
        GRAB_FIRST, SCORE_FIRST,
        GRAB_SECOND, SCORE_SECOND,
        PUSH_THIRD,
        PARK,
        STOP
    }

    private PathState pathState = PathState.SCORE_PRELOAD;

    private void generatePaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine( new Point(startPose), new Point(scorePose) )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(startPose.getHeading()), scorePose.getHeading()
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
        pushThirdSample = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(72.000 + OFFSET_X, 120.000 + OFFSET_Y, Point.CARTESIAN),
                                new Point(72.000 + OFFSET_X, 135.000 + OFFSET_Y, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(72.000 + OFFSET_X, 135.000 + OFFSET_Y, Point.CARTESIAN),
                                new Point(scorePushPose)
                        )
                )
                .setConstantHeadingInterpolation(scorePushPose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(scorePushPose),
                                new Point(60.000 + OFFSET_X, 128.000 + OFFSET_Y, Point.CARTESIAN),
                                new Point(parkPose)
                        )
                )
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }


    @Override
    public void init() {
        robot = new robot(hardwareMap);
        follower = new Follower(hardwareMap);
        
        generatePaths();

        follower.setStartingPose(startPose);
        Constants.setConstants(FConstants.class, LConstants.class);

        actionTimer = new Timer();
        

        telemetry.update();

        robot.intake.ManualLevel(universalValues.INTAKE_RETRACT, 0.8);
        robot.intake.setPivot(universalValues.INTAKE_INIT);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);

        robot.outtake.ManualLevel(universalValues.OUTTAKE_RETRACT, 0.8);
        robot.outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

    }

    @Override
    public void init_loop() {
        robot.intake.ManualLevel(universalValues.INTAKE_RETRACT, 0.8);
        robot.outtake.ManualLevel(universalValues.OUTTAKE_RETRACT, 0.8);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autoPathUpdate();
        //transfer();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }


    private void autoPathUpdate() {
        switch (pathState) {
            case START:
                robot.intake.setPivot(universalValues.INTAKE_INT);

                follower.followPath(scorePreload);
                changeAutoState(PathState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
                // CHECK for reaching position
                if ((follower.getPose().getX() > scorePose.getX() - 1) &&
                    (follower.getPose().getY() > scorePose.getY() - 1)) {
                    if (step == 0) {
                        actionTimer.resetTimer();
                        robot.outtake.ManualLevel(universalValues.OUTTAKE_EXTEND, 0.8);
                        robot.outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);
                        ++step;
                    }

                    if (step == 1 && actionTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
                        ++step;
                    }

                    if (step == 2 && actionTimer.getElapsedTimeSeconds() > 0.6) {
                        robot.outtake.ManualLevel(universalValues.OUTTAKE_RETRACT, 0.8);
                        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

                        follower.followPath(grabFirstSample);
                        changeAutoState(PathState.GRAB_FIRST);
                    }
                }
                break;

            case GRAB_FIRST:
                if ((follower.getPose().getX() > grabFirstSamplePose.getX() - 1) &&
                        (follower.getPose().getY() < grabFirstSamplePose.getY() + 1)) {
                    if (step == 0) {
                        actionTimer.resetTimer();
                        robot.intake.ManualLevel(universalValues.INTAKE_EXTEND, 0.8);
                        robot.intake.OpenIntake(universalValues.CLAW_OPEN);
                        robot.intake.setPivot(universalValues.INTAKE_DOWN);

                        ++step;
                    }

                    if (step == 1 && actionTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);

                        ++step;
                    }

                    if (step == 2 && actionTimer.getElapsedTimeSeconds() > 0.7) {
                        robot.intake.setPivot(universalValues.INTAKE_INT);

                        robot.universalTransfer.transfer();
                        follower.followPath(scoreFirstSample);
                        changeAutoState(PathState.SCORE_FIRST);
                    }
                }
                break;

            case SCORE_FIRST:
                // CHECK for reaching position
                if ((follower.getPose().getX() < scorePose.getX() + 1) &&
                        (follower.getPose().getY() < scorePose.getY() + 1)) {

                    if (!robot.universalTransfer.isTransferCompleted())
                        break;

                    if (step == 0) {
                        actionTimer.resetTimer();
                        robot.outtake.ManualLevel(universalValues.OUTTAKE_EXTEND, 0.8);
                        robot.outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);

                        ++step;
                    }

                    if (step == 1 && actionTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
                        ++step;
                    }

                    if (step == 2 && actionTimer.getElapsedTimeSeconds() > 0.6) {
                        robot.outtake.ManualLevel(universalValues.OUTTAKE_RETRACT, 0.8);
                        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

                        follower.followPath(grabSecondSample);
                        changeAutoState(PathState.GRAB_SECOND);
                    }
                }
                break;

            case GRAB_SECOND:
                if ((follower.getPose().getX() > grabSecodSamplePose.getX() - 1) &&
                        (follower.getPose().getY() < grabSecodSamplePose.getY() + 1)) {
                    if (step == 0) {
                        actionTimer.resetTimer();
                        robot.intake.ManualLevel(universalValues.INTAKE_EXTEND, 0.8);
                        robot.intake.OpenIntake(universalValues.CLAW_OPEN);
                        robot.intake.setPivot(universalValues.INTAKE_DOWN);

                        ++step;
                    }

                    if (step == 1 && actionTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);

                        ++step;
                    }

                    if (step == 2 && actionTimer.getElapsedTimeSeconds() > 0.7) {
                        robot.intake.setPivot(universalValues.INTAKE_INT);
                        robot.universalTransfer.transfer();

                        follower.followPath(scoreSecondSample);
                        changeAutoState(PathState.SCORE_SECOND);
                    }
                }
                break;

            case SCORE_SECOND:
                // CHECK for reaching position
                if ((follower.getPose().getX() < scorePose.getX() + 1) &&
                        (follower.getPose().getY() > scorePose.getY() - 1)) {
                    if (!robot.universalTransfer.isTransferCompleted())
                        break;

                    if (step == 0) {
                        actionTimer.resetTimer();
                        robot.outtake.ManualLevel(universalValues.OUTTAKE_EXTEND, 0.8);
                        robot.outtake.setPivot(universalValues.OUTTAKE_DUMP_BUCKET);

                        ++step;
                    }

                    if (step == 1 && actionTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);

                        ++step;
                    }

                    if (step == 2 && actionTimer.getElapsedTimeSeconds() > 0.6) {
                        robot.outtake.ManualLevel(universalValues.OUTTAKE_RETRACT, 0.8);
                        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT_NEW_TRANSFER);

                        follower.followPath(pushThirdSample);
                        changeAutoState(PathState.PUSH_THIRD);
                    }
                }
                break;

            case PUSH_THIRD:
                if ((follower.getPose().getX() < scorePose.getX() + 1) &&
                        (follower.getPose().getY() > scorePose.getY() + 1)) {

                    follower.followPath(park);
                    changeAutoState(PathState.PARK);
                }

            case PARK:
                if ((follower.getPose().getX() > parkPose.getX() - 1) &&
                        (follower.getPose().getY() < parkPose.getY() + 1)) {
                    changeAutoState(PathState.STOP);
                }

            default:
                break;
        }
    }

    private void changeAutoState(PathState state) {
        pathState = state;
        step = 0;
    }
}
