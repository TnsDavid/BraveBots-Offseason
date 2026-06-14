package org.firstinspires.ftc.teamcode.util.math;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.maxHoodPos;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Shooter.minHoodpos;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

public class ShooterMath {

    public static final double aVClose = -143.80411, bVClose = 909.21466, cVClose = 456.30415;
    public static final double aHClose = -0.0841684, bHClose = 0.490682, cHClose = 0.139603;

    public static double maxdegrees = 0; //nu stiu cat corespunde maximul la hood in grade
    public static double goalPosition = 0.98 ;
    public static double robotPosition = 0.25 ;

    public static double g = 9.81;
    public static double calculateHoodAngle(double dist){
        return Range.clip(aHClose * Math.pow(dist, 2) + bHClose * dist + cHClose, minHoodpos, maxHoodPos);
    }

    public static double calculateVelocity(double dist){
        return (aVClose * Math.pow(dist, 2) + bVClose * dist + cVClose);
    }

    public static double transforminMeters(double v){
        return ( v * Math.PI * 0.072 ) / 28 ;   //Nu stiu ticks/rev cat e si nu cred ca e 1550
    }
    public static double calculateTOF(double vel , double pos){
        return (transforminMeters(vel)* Math.sin(pos * maxdegrees) + Math.sqrt(Math.pow(transforminMeters(vel)* Math.sin(pos * maxdegrees), 2) + 2* g * (goalPosition -  robotPosition))) / g;
    }
}
