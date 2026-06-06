package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;

public class Turret {

    static double NEUTRAL_POSITION= .5;
    public static double x, y, heading, xCorner, yCorner, dist, fieldRelative, robotRelative, turretRelative, TOF, virtualDist;
    public static final double FIELD_LENGTH= 3.65, launchTime= .2,goalCircleRasius= .18;
    public static final double at = -0.467223, bt = 2.54517, ct = -2.64603;
    public static final double turretTOFCoef = 0.9, shooterTOFCoef = 1.05;
    public static BetterServoEx turret1, turret2;
    public static boolean reseted = false;

    public static void setNeutralPosition(double pos){
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }

    public static void updateAuto(Pose robotPose){
        if (robotPose.getX() != 0 && robotPose.getY() != 0) {

            x= robotPose.getX();
            y= robotPose.getY();
            heading= robotPose.getHeading();

            yCorner= FIELD_LENGTH- y;
            xCorner= Robot.a== Alliance.RED ? FIELD_LENGTH- x : x;
            dist = Math.hypot(xCorner, yCorner);

            heading= Math.toDegrees(heading);
            xCorner -= goalCircleRasius;
            yCorner -= goalCircleRasius;
            fieldRelative = Math.atan(yCorner / xCorner);
            fieldRelative = Math.toDegrees(fieldRelative);
            if (Robot.a == Alliance.BLUE)
                fieldRelative = 180 - fieldRelative;
            robotRelative = normalize(fieldRelative - heading);
            turretRelative = normalize(robotRelative - 180);
            robotRelative = Robot.a == Alliance.RED ? robotRelative : -robotRelative;
            turretRelative+= 180;
            double finalPos= turretRelative/360;
            if(finalPos>= 0.05 && finalPos<=0.95) {

                turret1.setPosition(finalPos);
                turret2.setPosition(finalPos);


            }



        }
    }

    //TODO: PUNETI SOTM;
    public static double offset = 0;
    public static void update(){
        Pose robotPose= Robot.robotPose;
        if (robotPose.getX() != 0 && robotPose.getY() != 0) {

            x= robotPose.getX();
            y= robotPose.getY();
            heading= robotPose.getHeading();

            yCorner= FIELD_LENGTH- y;
            xCorner= Robot.a== Alliance.RED ? FIELD_LENGTH- x : x;

            dist = Math.hypot(xCorner, yCorner);
//
//            double Vx = Robot.pinpoint.getVelX(DistanceUnit.METER);
//            double Vy = Robot.pinpoint.getVelY(DistanceUnit.METER);
//
//            if(dist < 3.3) TOF = at * Math.pow(dist, 2) + bt * dist + ct;
//            else TOF = 0.77;
//
//            xCorner -= Vx * TOF;
//            yCorner -= Vy * TOF;
//
//            virtualDist = Math.hypot(xCorner, yCorner);

            heading= Math.toDegrees(heading);
            xCorner -= goalCircleRasius;
            yCorner -= goalCircleRasius;
            fieldRelative = Math.atan(yCorner / xCorner);
            fieldRelative = Math.toDegrees(fieldRelative);
            if (Robot.a == Alliance.BLUE)
                fieldRelative = 180 - fieldRelative;
            robotRelative = normalize(fieldRelative - heading);
            turretRelative = normalize(robotRelative - 180);
            robotRelative = Robot.a == Alliance.RED ? robotRelative : -robotRelative;
            turretRelative+= 180;
            turretRelative+=offset;
            turretRelative = Range.clip(turretRelative,0,360);
            double finalPos= turretRelative/360;
            if(finalPos>= 0.1425 && finalPos<=0.91) {
                if(reseted) {
                    turret1.setPosition(finalPos);
                    turret2.setPosition(finalPos);
                }
                else{
                    turret1.setPosition(0.5);
                    turret2.setPosition(0.5);
                }
            }



        }
    }
    public static double normalize(double h){
        while (h> 180) h-=360;
        while (h< -180) h+= 360;
        return h;
    }
    public static double getPositon() {
        return turret1.getPosition();
    }

}