package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;




        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 10;

        FollowerConstants.xMovement = 64;
        FollowerConstants.yMovement = 48;

        FollowerConstants.forwardZeroPowerAcceleration = -28.373801;
        FollowerConstants.lateralZeroPowerAcceleration = -62.966;

        FollowerConstants.zeroPowerAccelerationMultiplier = 2.7;

        FollowerConstants.translationalPIDFSwitch = 0;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.22,0,0.027,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.015;
        
        FollowerConstants.headingPIDFSwitch = Math.PI / 20;
        FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.16,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.01;
        
        FollowerConstants.drivePIDFSwitch = 20;
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02,0,0.000005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.secondaryDrivePIDFFeedForward = 0;
        
        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
        
        FollowerConstants.APPROXIMATION_STEPS = 1000;
        FollowerConstants.holdPointTranslationalScaling = 0.45;
        FollowerConstants.holdPointHeadingScaling = 0.35;
        
        FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
        FollowerConstants.BEZIER_CURVE_BINARY_STEP_LIMIT = 10;
        
        
        
        
    }
}
