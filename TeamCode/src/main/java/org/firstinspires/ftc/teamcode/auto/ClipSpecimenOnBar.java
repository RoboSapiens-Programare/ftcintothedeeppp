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

@Autonomous(name = "4+0", group = "0. Autonomous")
public class ClipSpecimenOnBar extends OpMode {

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
    private List<DcMotorEx> motors;

    // TODO: modify y offset to correct pedro path visualiser offset

    private final Pose startPose = new Pose(8.5,49, Math.toRadians(180));
    private final Pose behindSample1 = new Pose(65, 29.500, Math.toRadians(180));
    private final Pose pushSample1 = new Pose(18.272, 29.400, Math.toRadians(180));
    private final Pose behindSample2 = new Pose(64.760, 19.069, Math.toRadians(180));
    private final Pose pushSample2 = new Pose(18.272, 18.620, Math.toRadians(180));
    private final Pose specimenPickup1 = new Pose(20.1, 29, Math.toRadians(180));
    private final Pose barCliponPose1 = new Pose(35.75,67.5, Math.toRadians(180));
    private final Pose barCliponPose2 = new Pose(36.5,74.5, Math.toRadians(180));
    private final Pose barCliponPose3 = new Pose(36.5, 78.5, Math.toRadians(180));
    private final Pose barCliponPose4 = new Pose(36.7, 83.5, Math.toRadians(180));
    private final Pose ParkPose = new Pose(2,32, Math.toRadians(180));
    private final Pose behindSample3 = new Pose(64.535, 12.556, Math.toRadians(180));
    private final Pose pushSample3 = new Pose(18.272, 12.332, Math.toRadians(180));


    private PathChain toBar1, toSample1, toHuman1, toSample2, toHuman2, toSpecimenPickup1, toBar2, toSpecimenPickup2,toSpecimenPickup3, toBar3, park,toSample3, toHuman3, toBar4;

    public void buildPaths() {
        // TODO : fix Linear Heading Interpolation for rotation to the right without hard coding value bigger than 180 (line 37)
        toBar1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(barCliponPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSample1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(barCliponPose1),
                                new Point(29.420, 39.875, Point.CARTESIAN),
                                new Point(38.628, 34.485, Point.CARTESIAN),
                                new Point(61.535, 42.404, Point.CARTESIAN),
                                new Point(behindSample1)
                        )
                )

                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toHuman1 = follower.pathBuilder()
                //.setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierLine(new Point(behindSample1), new Point(pushSample1.getX()-50, pushSample1.getY())))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSample2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pushSample1),
                                new Point(74.866, 32.544, Point.CARTESIAN),
                                new Point(behindSample2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toHuman2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSample2), new Point(pushSample2.getX() - 50, pushSample2.getY())))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSpecimenPickup1 = follower.pathBuilder()

                .addPath(new BezierLine(new Point(pushSample2), new Point(25, 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(25, 23, Point.CARTESIAN), new Point(specimenPickup1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        toBar2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickup1), new Point(barCliponPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSpecimenPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(new Point(barCliponPose2), new Point(specimenPickup1)))

//                .addPath(new BezierLine(new Point(18, 57, Point.CARTESIAN), new Point(19.500, 27.600, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSpecimenPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(new Point(barCliponPose4), new Point(specimenPickup1)))

//                .addPath(new BezierLine(new Point(18, 57, Point.CARTESIAN), new Point(19.500, 27.600, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toBar3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickup1), new Point(barCliponPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toBar4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickup1), new Point(barCliponPose4)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(barCliponPose3), new Point(ParkPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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
                follower.followPath(toBar1,true);
                setPathState(1);
                break;
            case(1):
                if (singleton2)
                {
                    robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP_BUCKET);
                    singleton2 = false;
                }

                if((follower.getPose().getX() > (barCliponPose1.getX() - 1) && follower.getPose().getY() > (barCliponPose1.getY() - 1))) {
                    if (singleton)
                    {
                        stateTimer.resetTimer();
                        singleton = false;
                    }
                    robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                    if (stateTimer.getElapsedTimeSeconds()>0.3)
                    {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        if (stateTimer.getElapsedTimeSeconds()>0.4)
                        {
                            robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                            robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                            follower.followPath(toSample1,true);
                            setPathState(2);
                        }
                    }

                }
                break;
            case(2):
                if((follower.getPose().getX() > (behindSample1.getX() - 1) && follower.getPose().getY() < (behindSample1.getY() + 1)))
                {
                    follower.followPath(toHuman1, true);
                    setPathState(3);
                }
                break;
            case(3):
                if((follower.getPose().getX() < (pushSample1.getX() + 1) && follower.getPose().getY() < (pushSample1.getY() + 1)))
                {
                    follower.followPath(toSample2, true);
                    setPathState(4);
                }
                break;
            case(4):
                if((follower.getPose().getX() > (behindSample2.getX() - 1) && follower.getPose().getY() > (behindSample2.getY() - 1)))
                {
                    follower.followPath(toHuman2, true);
                    setPathState(5);
                }
                break;
            case(5):
                if((follower.getPose().getX() < (pushSample2.getX() + 1) && follower.getPose().getY() < (pushSample2.getY() + 1)))
                {
                        follower.followPath(toSpecimenPickup1, true);
                        robot.intake.setPivot(UniversalValues.INTAKE_INT);
                        robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                        setPathState(6);
                }
                break;
            case(6):


                if((follower.getPose().getX() < (specimenPickup1.getX() + 1) && follower.getPose().getY() < (specimenPickup1.getY() + 1)))
                {


                    if (singleton)
                    {

                        stateTimer.resetTimer();
                        robot.intake.setClawPivot(CLAW_HORIZONTAL);
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
                        if (stateTimer.getElapsedTimeSeconds() > 2.25) {
                            robot.intake.setClawPivot(0.6);
                            robot.intake.setPivot(UniversalValues.INTAKE_UP);

                            follower.followPath(toBar2, true);
                            setPathState(7);
                        }
                    }

                }
                break;
            case(7):
                robot.universalTransfer.transfer();
                if((follower.getPose().getX() > (barCliponPose2.getX() - 1) && follower.getPose().getY() > (barCliponPose2.getY() - 1))) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton2)
                        {
                            stateTimer.resetTimer();
                            singleton2 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                                    follower.followPath(toSpecimenPickup2, true);

                                    robot.universalTransfer.resetTransfer();
                                    setPathState(8);
                                }
                            }
                        }
                    }
                }
                break;
            case(8):


                if((follower.getPose().getX() < (specimenPickup1.getX() + 1) && follower.getPose().getY() < (specimenPickup1.getY() + 1))) {


                    if (singleton) {

                        robot.intake.setClawPivot(CLAW_HORIZONTAL);
                        stateTimer.resetTimer();
                        singleton = false;
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 0.7) {
                        singleton3 = false;
                        robot.intake.ManualLevel(INTAKE_RETRACT + 150, 1);
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 1.1) {
                        robot.intake.OpenIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 1.3) {
                            robot.intake.setClawPivot(0.6);
                            robot.intake.setPivot(UniversalValues.INTAKE_UP);

                            if (stateTimer.getElapsedTimeSeconds() > 1.4) {
                                follower.followPath(toBar3, true);
                                setPathState(9);
                            }
                        }
                    }


                }
                break;
            case(9):
                robot.universalTransfer.transfer();
                if((follower.getPose().getX() > (barCliponPose3.getX() - 1) && follower.getPose().getY() > (barCliponPose3.getY() - 1))) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton2)
                        {
                            stateTimer.resetTimer();
                            singleton2 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 1);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
                                    follower.followPath(toSpecimenPickup3, true);

                                    robot.universalTransfer.resetTransfer();
                                    setPathState(10);
                                }
                            }
                        }
                    }
                }
                break;
            case(10):


                if((follower.getPose().getX() < (specimenPickup1.getX() + 1) && follower.getPose().getY() < (specimenPickup1.getY() + 1))) {


                    if (singleton) {


                        robot.intake.setClawPivot(CLAW_HORIZONTAL);
                        stateTimer.resetTimer();
                        singleton = false;
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 0.7) {
                        singleton3 = false;
                        robot.intake.ManualLevel(INTAKE_RETRACT + 150, 1);
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 1.1) {
                        robot.intake.OpenIntake(CLAW_CLOSE);
                        if (stateTimer.getElapsedTimeSeconds() > 1.3) {
                            robot.intake.setClawPivot(0.6);
                            robot.intake.setPivot(UniversalValues.INTAKE_UP);

                            if (stateTimer.getElapsedTimeSeconds() > 1.4) {
                                follower.followPath(toBar4, true);
                                setPathState(11);
                            }
                        }
                    }


                }
                break;
            case(11):
                robot.universalTransfer.transfer();
                if((follower.getPose().getX() > (barCliponPose4.getX() - 1) && follower.getPose().getY() > (barCliponPose4.getY() - 1))) {
                    if (robot.universalTransfer.isTransferCompleted()) {
                        if (singleton2)
                        {
                            stateTimer.resetTimer();
                            singleton2 = false;
                        }

                        if (stateTimer.getElapsedTimeSeconds() > 0.75) {
                            robot.outtake.ManualLevel(OUTTAKE_EXTEND_SPECIMEN, 1);
                            if (stateTimer.getElapsedTimeSeconds() > 1) {
                                robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                                if (stateTimer.getElapsedTimeSeconds() > 1.15) {
                                    robot.outtake.ManualLevel(OUTTAKE_RETRACT, 0.7);
                                    robot.outtake.setPivot(OUTTAKE_COLLECT_NEW_TRANSFER);
//                                    follower.followPath(toSpecimenPickup3, true);

                                    robot.universalTransfer.resetTransfer();

                                    setPathState(-1);
                                }
                            }
                        }
                    }
                }
                break;
        }
    }


    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);

        robot = new Robot(hardwareMap);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

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
    }

    @Override
    public void stop() {
    }
}