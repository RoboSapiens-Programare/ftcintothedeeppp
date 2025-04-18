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

        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.mass = 10;

        FollowerConstants.xMovement = 75.34096617148526;
        FollowerConstants.yMovement = 55.33304819533794;

        FollowerConstants.forwardZeroPowerAcceleration = -23.667635870564364;
        FollowerConstants.lateralZeroPowerAcceleration = -65.57724187428536;

        FollowerConstants.zeroPowerAccelerationMultiplier = 2;

        FollowerConstants.translationalPIDFSwitch = 0;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.41,0,0.03,0);

        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.015;
        
        FollowerConstants.headingPIDFSwitch = Math.PI / 20;
        FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.16,0);

        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.01;
        
        FollowerConstants.drivePIDFSwitch = 20;
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0005,0.6,0);

        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.secondaryDrivePIDFFeedForward = 0;
        
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
        
        
        
        
    }
}
