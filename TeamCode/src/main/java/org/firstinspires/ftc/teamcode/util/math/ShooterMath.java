package org.firstinspires.ftc.teamcode.util.math;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.maxHoodPos;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.minHoodpos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

@Configurable
public class ShooterMath {
    public static double aVFar = -32.30065, bVFar = 341.02654, cVFar = 347.6482;
    public static final double aHFar = 0.004, bHFar = -0.0882, cHFar = 0.85144;
    //y=160x+572
    public static final double aVClose = 0, bVClose = 160, cVClose = 572;
    //y=0.0126857x^{2}+0.0716514x+0.25198
    public static final double aHClose = 0.0126857, bHClose = 0.0716514, cHClose = 0.25198;
    //y=\left(4.95536\times10^{-7}\right)x^{2}-0.000119071x+0.157958
    public static final double aHClose2 = 4.95536e-7, bHClose2 = -0.000119071, cHClose2 = 0.157958;

    public static double TOFCoef = 0.85;
    public static double TOFCoefFar = 0.75;

    public static double maxdegrees = Math.toRadians(60);
    public static double goalPosition = 0.98 ;
    public static double robotPosition = 0.25 ;

    public static double g = 9.81;
    public static double calculateHoodAngleFromDistance(double dist){
        return Range.clip(aHClose * Math.pow(dist, 2) + bHClose * dist + cHClose, minHoodpos, maxHoodPos);
    }
    public static double calculateHoodAngleFromVelocity(double velo) {
        return Range.clip(aHClose2 * Math.pow(velo, 2) + bHClose2 * velo + cHClose2, minHoodpos, maxHoodPos);
    }
    public static double  calculateVelocityFromDistance(double dist){
        return (aVClose * Math.pow(dist, 2) + bVClose * dist + cVClose);
    }
    public static double calculateVelocityFARFromDistance(double dist){
        return (aVFar * Math.pow(dist, 2) + bVFar * dist + cVFar);
    }
    public static double calculateHoodFARFromDistance(double dist){
        return (aHFar * Math.pow(dist, 2) + bHFar * dist + cHFar);
    }

    public static double transformInMeters(double v){
        return ( v * Math.PI * 0.072 ) / 28;
    }
    public static double calculateTOF(double vel, double pos){
        double velocityTransformed = transformInMeters(vel);
        double positionTransformed = pos * maxdegrees;
        double verticalSpeed = velocityTransformed * Math.sin(positionTransformed);
        return ((verticalSpeed + Math.sqrt(Math.pow(verticalSpeed, 2) + 2 * g * (goalPosition - robotPosition)) ) / g ) * (Robot.playStyle == Robot.PlayStyle.CLOSE ? TOFCoef : TOFCoefFar);
    }
}
