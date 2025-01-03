package pedroPathing.auto;

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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystems.robot;
import pedroPathing.subsystems.universalValues;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Push Specimen", group = "Autonomous")
public class ParkSpecimen extends OpMode {
    private pedroPathing.subsystems.robot robot = null;
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(8,49, Math.toRadians(0)); // visualizer: (x: 10 y: 46)
    private boolean singleton = true;
    private PathChain samplePush;

    private void generatePaths() {
        samplePush = follower.pathBuilder()

                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(8.000, 63.000, Point.CARTESIAN),
                                new Point(16.168, 24.757, Point.CARTESIAN),
                                new Point(45.103, 50.551, Point.CARTESIAN),
                                new Point(70.000, 28.000, Point.CARTESIAN)
                        )
                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(70.000, 28.000, Point.CARTESIAN),
                                new Point(8.000, 28.000, Point.CARTESIAN)
                        )
                )
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(8.000, 28.000, Point.CARTESIAN),
                                new Point(61.925, 32.159, Point.CARTESIAN),
                                new Point(70.000, 17.000, Point.CARTESIAN)
                        )
                )
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(70.000, 17.000, Point.CARTESIAN),
                                new Point(8.000, 17.000, Point.CARTESIAN)
                        )
                )
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(8.000, 17.000, Point.CARTESIAN),
                                new Point(75.832, 22.065, Point.CARTESIAN),
                                new Point(70.000, 12.000, Point.CARTESIAN)
                        )
                )
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(70.000, 12.000, Point.CARTESIAN),
                                new Point(8.000, 12.000, Point.CARTESIAN)
                        )
                )
                .build();
    }




    @Override
    public void init() {
        robot = new robot(hardwareMap);


        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        Constants.setConstants(FConstants.class, LConstants.class);

        telemetry.update();

        // buildPaths();
        generatePaths();

        // Set the subsystems to positions for init
        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(universalValues.INTAKE_INIT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
    }

    @Override
    public void loop() {
        if (singleton) {
            follower.followPath(samplePush);
            singleton = false;
        }

        follower.update();
    }
}