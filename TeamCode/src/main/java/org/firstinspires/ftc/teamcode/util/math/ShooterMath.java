package org.firstinspires.ftc.teamcode.util.math;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.maxHoodPos;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.minHoodpos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

@Configurable
public class ShooterMath {

    public static final double aVClose = -116.65867, bVClose = 800.80404, cVClose = 386.81867;
    public static final double aHClose = -0.101926, bHClose = 0.585852, cHClose = -0.0537098;
    public static double TOFCoef = 1;

    public static double maxdegrees = Math.toRadians(60);
    public static double goalPosition = 0.98 ;
    public static double robotPosition = 0.25 ;

    public static double g = 9.81;
    public static double calculateHoodAngle(double dist){
        return Range.clip(aHClose * Math.pow(dist, 2) + bHClose * dist + cHClose, minHoodpos, maxHoodPos);
    }

    public static double  calculateVelocity(double dist){
        return (aVClose * Math.pow(dist, 2) + bVClose * dist + cVClose);
    }

    public static double transformInMeters(double v){
        return ( v * Math.PI * 0.072 ) / 28;   //Nu stiu ticks/rev cat e si nu cred ca e 1550
    }
    public static double calculateTOF(double vel, double pos){
        double velocityTransformed = transformInMeters(vel);
        double positionTransformed = pos * maxdegrees;
        double verticalSpeed = velocityTransformed * Math.sin(positionTransformed);
        return ((verticalSpeed +
                Math.sqrt(Math.pow(verticalSpeed, 2) + 2 * g * (goalPosition - robotPosition)) ) / g ) * TOFCoef;

    }
}
