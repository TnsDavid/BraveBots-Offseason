package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.MathStuff;

public class Constants {
    public static double dtLength =0.435;
    public static double dtWidth = 0.432;
    public static double xOffset = -4.02, yOffset = 0.944;//0.98

    public static final double odometryResolution = 34.310551 * 25.4; //ticks per mm *
    public static double kp = 0.06, kquad = 0.0018583773298043212, klinear = 0.06142244021388717;
    public static double p = 0, i = 0, d = 0, f= 0;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.48)
            .forwardZeroPowerAcceleration(-45.17)
            .lateralZeroPowerAcceleration(-65.64)
           // .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(kp,klinear,kquad))
            .centripetalScaling(0.00097)
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.06, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0072 , 0 , 0.00058 , 0 , 0.14))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.01,0.028 ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15 , 0 , 0.02 ,0.028 ) )
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(true);


    public static PathConstraints pathConstraints = new PathConstraints(0.97, 0, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(xOffset)
            .strafePodX(yOffset)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")//FL
            .rightRearMotorName("BR")//BL
            .leftRearMotorName("BL")//BR
            .leftFrontMotorName("FL")//FR
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .useVoltageCompensation(true)
            .xVelocity(83.75)
            .yVelocity(64.85)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}