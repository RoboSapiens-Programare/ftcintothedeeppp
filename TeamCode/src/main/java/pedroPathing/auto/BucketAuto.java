package pedroPathing.auto;


import static pedroPathing.subsystems.universalValues.INTAKE_EXTEND;
import static pedroPathing.subsystems.universalValues.OUTTAKE_CLOSE;
import static pedroPathing.subsystems.universalValues.OUTTAKE_DUMP_BUCKET;
import static pedroPathing.subsystems.universalValues.OUTTAKE_OPEN;

import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import pedroPathing.subsystems.robot;
import pedroPathing.subsystems.universalValues;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Bucket Auto", group = "Autonomous")
public class BucketAuto extends OpMode {
private pedroPathing.subsystems.robot robot = null;
private Follower follower;
private Timer pathTimer, actionTimer, opmodeTimer, actionTimer2, transferTimer;
private boolean singleton = true;
private boolean singleton2 = true;

/** This is the variable where we store the state of our auto.
 * It is used by the pathUpdate method. */
private int pathState;

/** Create and Define Poses + Paths
 * Poses are built with three constructors: x, y, and heading (in Radians).
 * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
 * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
 * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
 * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/> */

//TODO : update paths with correct poses and/or bezier curves

private final Pose startPose = new Pose(8,103, Math.toRadians(-90));
private final Pose scorePose = new Pose(6.5,127, Math.toRadians(315));
private final Pose pickup1Pose = new Pose(10, 126, Math.toRadians(0));
private final Pose pickup2Pose = new Pose(10, 117, Math.toRadians(0));
private final Pose intpushPose = new Pose(72.6, 118.6, Math.toRadians(-90));
private final Pose pushpose = new Pose(8, 130, Math.toRadians(-90));
private final Pose parkPose = new Pose(61,90, Math.toRadians(-90));
private final Pose parkEmergencyPose = new Pose(14,29, Math.toRadians(-90));
private boolean transfersingleton1 = true;
private boolean transfersingleton2 = true;
private boolean transfersingleton3 = true;
private boolean transfersingleton4 = true;
private boolean transfersingleton5 = true;
private boolean transfersingleton6 = true;
private boolean transfersingleton7 = true;
private boolean isSpecimeninClaw = false;
private boolean isTransferDone = false;
private boolean isPressed = false;

private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, intpushPickup3,pushPickup3, park, scorePreload;

/** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
 * It is necessary to do this so that all the paths are built before the auto starts. **/

public void buildPaths() {
    //score preload path



scorePreload = follower.pathBuilder()
        .addPath(new BezierCurve(new Point(startPose),new Point(28.935, 119.103, Point.CARTESIAN), new Point(scorePose)))
        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
        .build();


    //grab pickup 1 path
grabPickup1 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
        .build();

    //score pickup 1 path
scorePickup1 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())


        .build();

    //grab pickup 2 path
grabPickup2 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
        .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
        .build();

    //score pickup 2 path
scorePickup2 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
        .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
        .build();

//intermediary push path
intpushPickup3 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(scorePose), new Point(intpushPose)))
        .setLinearHeadingInterpolation(scorePose.getHeading(), intpushPose.getHeading())
        .build();

    //push pickup 3 path
pushPickup3 = follower.pathBuilder()
        .addPath(new BezierCurve(new Point(intpushPose),new Point(98.5, 134.5, Point.CARTESIAN), new Point(pushpose)))
        .setLinearHeadingInterpolation(intpushPose.getHeading(), pushpose.getHeading())
        .build();


    //park path
park = follower.pathBuilder()
        .addPath(new BezierCurve(new Point(pushpose),new Point(82, 122.5, Point.CARTESIAN), new Point(parkPose)))
        .setLinearHeadingInterpolation(pushpose.getHeading(), parkPose.getHeading())
        .build();
}



/** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
 * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
 * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
public void autonomousPathUpdate() {

    //TODO : fix useless repeating instructions and ispressed boolean logic

    switch (pathState) {

        case 0:

            //GOES TO SCORE POSITION
            robot.intake.setPivot(universalValues.INTAKE_INT);
            follower.followPath(scorePreload,true);
            actionTimer.resetTimer();
            setPathState(1);
            break;

        case 1:

            //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES PRELOAD, GOES TO GRABBING FIRST SAMPLE POSITION AND HOLDS
            if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                robot.outtake.ManualLevel(universalValues.OUTTAKE_EXTEND, 1);


                if (actionTimer.getElapsedTimeSeconds() > 3)
                {

                    if (singleton2)
                    {
                        robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
                        singleton2 = false;
                    }

                    if (actionTimer.getElapsedTimeSeconds() > 4)
                    {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        if (actionTimer.getElapsedTimeSeconds() > 6)
                        {
                            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);

                            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);

                            if (actionTimer.getElapsedTimeSeconds() > 7)
                            {
                                setPathState(-1);
                                isTransferDone = false;
                                isSpecimeninClaw = false;
                                robot.outtake.ManualLevel(0, 0.4);
                            }
                        }

                    }
                }
            }
            break;

        case 2:
            if (!singleton)
            {
                transfer();
                if (isTransferDone)
                {
                    setPathState(3);

                }
            }

            //CHECKS IF ROBOT IS AT GRAB POSITION, GRABS SAMPLE, GOES TO SCORING POSITION AND HOLDS
            if(follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {

                singleton = false;

                if (isSpecimeninClaw && singleton2)
                {
                    singleton2 = false;
                    follower.followPath(scorePickup1, true);
                }
                            }
            break;
        case 3:

            //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES SAMPLE, GOES TO GRABBING SECOND SAMPLE POSITION AND HOLDS
            if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                robot.outtake.ManualLevel(universalValues.OUTTAKE_EXTEND, 1);


                if (actionTimer.getElapsedTimeSeconds() > 1)
                {
                    if (singleton2){
                    robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
                    singleton2 = false;
                }
                    if (actionTimer.getElapsedTimeSeconds() > 2)
                    {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        if (actionTimer.getElapsedTimeSeconds() > 4)
                        {

                            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
                            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);

                            if (actionTimer.getElapsedTimeSeconds() > 5)
                            {
                                follower.followPath(grabPickup2);
                                setPathState(4);
                                isTransferDone = false;
                                isSpecimeninClaw = false;
                                robot.outtake.ManualLevel(0, 0.4);
                            }
                        }

                    }
                }
            }
            break;

        case 4:

            if (!singleton)
            {
                transfer();
                if (isTransferDone)
                {
                    setPathState(5);
                }
            }

            //CHECKS IF ROBOT IS AT GRAB POSITION, GRABS SAMPLE, GOES TO SCORING POSITION AND HOLDS
            if(follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {

                singleton = false;

                if (isSpecimeninClaw)
                {
                    follower.followPath(scorePickup2, true);
                }
            }
            break;

        case 5:

            //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES SAMPLE, GOES TO GRABBING THIRD SAMPLE POSITION AND HOLDS
            if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                robot.outtake.ManualLevel(universalValues.OUTTAKE_EXTEND, 1);


                if (actionTimer.getElapsedTimeSeconds() > 3)
                {

                    robot.outtake.setPivot(OUTTAKE_DUMP_BUCKET);
                    if (actionTimer.getElapsedTimeSeconds() > 4)
                    {
                        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
                        if (actionTimer.getElapsedTimeSeconds() > 6)
                        {
                            follower.followPath(intpushPickup3);
                            setPathState(6);
                            isTransferDone = false;
                            isSpecimeninClaw = false;
                            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
                            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
                            robot.outtake.ManualLevel(0, 0.4);
                        }

                    }
                }
            }
            break;

        case 6:
            if(follower.getPose().getX() > (intpushPose.getX() - 1) && follower.getPose().getY() > (intpushPose.getY() - 1)) {
                follower.followPath(park, /* holdEnd = */ true);
                setPathState(8);
                }
            break;

        case 7:
            if((follower.getPose().getX() < (pushpose.getX() + 1) && follower.getPose().getY() > (pushpose.getY() - 1))) {
                follower.followPath(park, /* holdEnd = */ true);
                robot.intake.setPivot(0.55);
                setPathState(8);
            }
            break;

        //CHECKS IF ROBOT IS AT PARKING POSITION, EXTEND SLIDES AT LEVEL 1 ASCENT POSITION
        case 8:
            robot.intake.setPivot(0.55);
            if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                robot.outtake.ManualLevel(0,0.75);
                robot.intake.ManualLevel(0,1);

                robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);

                robot.intake.CloseIntake(universalValues.CLAW_CLOSE);

                singleton2 = true;
                singleton = true;
                setPathState(-1);
            }
            break;

        //EMERGENCY PARK CASE
        case 9:
            robot.intake.ManualLevel(0,1);
            robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
            robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
            robot.intake.setPivot(universalValues.INTAKE_INIT);
            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
            robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
            PathChain parkEmergency;
            parkEmergency = follower.pathBuilder()
                    .setZeroPowerAccelerationMultiplier(8)
                    .addPath(new BezierLine(new Point(follower.getPose()), new Point(parkEmergencyPose)))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkEmergencyPose.getHeading())
                    .build();
            follower.followPath(parkEmergency, /* holdEnd = */ true);
            break;
    }

    }

    public void transfer() {
        if (transfersingleton1)
        {
            robot.intake.OpenIntake(universalValues.CLAW_OPEN-0.275);
            robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);


            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
            transfersingleton1 = false;
        }
        if (transferTimer.getElapsedTimeSeconds() > 1)
        {
            if (transfersingleton7)
            {
                robot.intake.ManualLevel(INTAKE_EXTEND,1);
                robot.intake.setPivot(universalValues.INTAKE_DOWN);
                transfersingleton7 = false;
            }

        }
        if (transferTimer.getElapsedTimeSeconds() > 2.5)
        {
            if (transfersingleton2)
            {
                robot.intake.CloseIntake(universalValues.CLAW_CLOSE - 0.05);

                transfersingleton2 = false;
            }
            if (transferTimer.getElapsedTimeSeconds() > 3.25)
            {
                if (transfersingleton3) {
                    robot.intake.setPivot(universalValues.INTAKE_UP);
                    robot.outtake.CloseOuttake(universalValues.OUTTAKE_OPEN);
                    isSpecimeninClaw = true;
                    transfersingleton3 = false;
                }
                if (transferTimer.getElapsedTimeSeconds() > 3.25)
                {
                    if (transfersingleton4) {
                        robot.intake.ManualLevel(0, 1);
                        transfersingleton4 = false;

                    }
                    if (transferTimer.getElapsedTimeSeconds() > 4.25)
                    {
                        if (transfersingleton6)
                        {
                            robot.outtake.OpenOuttake(universalValues.OUTTAKE_CLOSE-0.075);
                            transfersingleton6 = false;
                        }
                        if (transferTimer.getElapsedTimeSeconds() > 6)
                        {
                            robot.intake.OpenIntake(universalValues.CLAW_OPEN);
                            if (transferTimer.getElapsedTimeSeconds() > 6)
                            {
                                if (transfersingleton5) {
                                    robot.intake.ManualLevel(300, 1);

                                    transfersingleton5 = false;
                                }
                                robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
                                if (transferTimer.getElapsedTimeSeconds() > 6.25)
                                {
                                    robot.intake.setPivot(universalValues.INTAKE_DOWN+0.2);
                                    if (transferTimer.getElapsedTimeSeconds() > 6.75)
                                    {
                                        robot.intake.ManualLevel(0, 1);
                                        isTransferDone = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }


/** These change the states of the paths and actions
 * It will also reset the timers of the individual switches **/
public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
    transferTimer.resetTimer();
    singleton = true;
    singleton2 = true;
    actionTimer.resetTimer();
    actionTimer2.resetTimer();
    transfersingleton1 = true;
    transfersingleton2 = true;
    transfersingleton3 = true;
    transfersingleton4 = true;
    transfersingleton5 = true;
    transfersingleton6 = true;
    transfersingleton7 = true;

}

/** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
@Override
public void loop() {

    // These loop the movements of the robot
    follower.update();
    autonomousPathUpdate();
    //transfer();
    // Feedback to Driver Hub
    telemetry.addData("path state", pathState);
    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
}

/** This method is called once at the init of the OpMode. **/
@Override
public void init() {
    robot = new robot(hardwareMap);
    pathTimer = new Timer();
    actionTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    actionTimer2 = new Timer();
    actionTimer2.resetTimer();
    transferTimer = new Timer();
    transferTimer.resetTimer();

    follower = new Follower(hardwareMap);
    follower.setStartingPose(startPose);
    Constants.setConstants(FConstants.class, LConstants.class);


    telemetry.update();

    buildPaths();


    // Set the subsystems to positions for init
    robot.intake.ManualLevel(0,1);
    robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
    robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
    robot.intake.setPivot(universalValues.INTAKE_INIT);
    robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
    robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
}

/** This method is called continuously after Init while waiting for "play". **/
@Override
public void init_loop() {
    opmodeTimer.resetTimer();
    actionTimer.resetTimer();
    actionTimer2.resetTimer();
    transferTimer.resetTimer();
}

/** This method is called once at the start of the OpMode.
 * It runs all the setup actions, including building paths and starting the path system **/
@Override
public void start() {
    opmodeTimer.resetTimer();
    setPathState(0);
}

/** We do not use this because everything should automatically disable **/
@Override
public void stop() {
}
}
