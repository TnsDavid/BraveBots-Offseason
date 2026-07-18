package org.firstinspires.ftc.teamcode.util.math;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.maxHoodPos;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.minHoodpos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

@Configurable
public class ShooterMath {

    public static final double aVClose = 0, bVClose = 240, cVClose = 960;
    public static final double aVFar = -136.05442, bVFar = 1111.56463, cVFar = -302.20408;
    public static final double aHClose = -0.0147429, bHClose = 0.22292, cHClose = 0.295363;
    public static final double aHClose2 = -0.000000255952, bHClose2 = 0.00142026, cHClose2 = -0.832203;

    public static double TOFCoef = 1;

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

    public static double transformInMeters(double v){
        return ( v * Math.PI * 0.072 ) / 28;
    }
    public static double calculateTOF(double vel, double pos){
        double velocityTransformed = transformInMeters(vel);
        double positionTransformed = pos * maxdegrees;
        double verticalSpeed = velocityTransformed * Math.sin(positionTransformed);
        return ((verticalSpeed +
                Math.sqrt(Math.pow(verticalSpeed, 2) + 2 * g * (goalPosition - robotPosition)) ) / g ) * TOFCoef;

    }
}
