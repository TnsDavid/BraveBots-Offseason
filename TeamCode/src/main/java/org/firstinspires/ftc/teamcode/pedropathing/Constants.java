package org.firstinspires.ftc.teamcode.pedropathing;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Scheduler;
import org.firstinspires.ftc.teamcode.util.math.MathStuff;
@Configurable
public class Constants {
    public static double dtLength =0.435;
    public static double dtWidth = 0.432;
    public static double xOffset = -4.02, yOffset = 0.944;

    public static final double odometryResolution = 34.310551 * 25.4; //ticks per mm *
     public static double kp = 0.045, kquad = 0.0015109948145492942, klinear = 0.0925188490735762;
    public static double p = 0, i = 0, d = 0, f= 0;

    public static double nominalvoltage = 10.2;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.56)
            .forwardZeroPowerAcceleration(-24.38)
            .lateralZeroPowerAcceleration(-60.53)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(kp,klinear,kquad).withMaximumBrakingPower(0.2))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.06, 0.01))
            .centripetalScaling(0)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.01,0.028))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15 , 0 , 0.02 ,0.028 ) )
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(true);


    public static PathConstraints pathConstraints = new PathConstraints(0.97, 0, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(yOffset)
            .strafePodX(xOffset)
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
            .nominalVoltage(nominalvoltage)
            .xVelocity(86.3627)
            .yVelocity(67.8612)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}