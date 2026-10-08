package org.firstinspires.ftc.teamcode.util.math;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.maxHoodPos;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.minHoodpos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.globals.Robot;

@Configurable
public class ShooterMath {
    public static double aVFar = 0, bVFar = 160, cVFar = 536;
    public static final double aHFar = 0.004, bHFar = -0.0882, cHFar = 0.85144;
    public static final double aVClose = 0, bVClose = 160, cVClose = 552;
    public static final double aHClose = 0.180343, bHClose = -0.433714, cHClose = 0.623712;
    public static final double aHClose2 = 0.00000704464, bHClose2 = -0.010488, cHClose2 = 4.26656;

    public static double TOFCoef = 0.85;
    public static double TOFCoefFar = 0.6;

    public static double maxdegrees = Math.toRadians(60);
    public static double goalPosition = 0.98 ;
    public static double robotPosition = 0.25 ;

    public static double g = 9.81;
    public static double calculateHoodAngleFromDistance(double dist){
        return Range.clip(aHClose * Math.pow(dist, 2) + bHClose * dist + cHClose, minHoodpos, maxHoodPos);
    }
    public static double calculateHoodAngleFromVelocity(double velo) {
        velo = Math.abs(velo);
        return Range.clip(aHClose2 * Math.pow(velo, 2) + bHClose2 * velo + cHClose2, minHoodpos, maxHoodPos);
    }
    public static double calculateVelocityFromDistance(double dist){
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
