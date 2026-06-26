package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.MathStuff;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;

@Configurable
public class Turret {

    static double NEUTRAL_POSITION= .5;
    public static double x, y, heading, xCorner, yCorner, dist, fieldRelative, robotRelative, turretRelative, TOF, virtualDist;
    public static final double FIELD_LENGTH= 3.65, launchTime= .2,goalCircleRasius= .18;
    //y=-0.0239556x^{2}+0.242631x+0.322645
    public static final double at = -0.0275109, bt = 0.253576, ct = 0.273474;
    public static double turretTOFCoef = 1, shooterTOFCoef = 1.05;
    public static BetterServoEx turret1, turret2;
    public static boolean reseted = false;
    public static boolean sotm = false;
    public static boolean checkedStatus = false;

    public enum TurretStatus {
        NOTACTIVE, ACTIVE
    }

    public static TurretStatus turrStatus = TurretStatus.NOTACTIVE;

    public static void toggleStatus(){
        turrStatus = turrStatus.equals(TurretStatus.ACTIVE) ? TurretStatus.NOTACTIVE : TurretStatus.ACTIVE;
    }
    public static void setNeutralPosition(double pos){
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }

    public static void updateAuto(Follower f , double offset){
        Pose robotPose = new Pose(f.getPose().getX() * 0.0254, f.getPose().getY() * 0.0254, f.getHeading());
        if (robotPose.getX() != 0 && robotPose.getY() != 0) {

            x= robotPose.getX();
            y= robotPose.getY();
            heading= robotPose.getHeading();

            yCorner= FIELD_LENGTH- y;
            xCorner= Robot.a== Alliance.RED ? FIELD_LENGTH- x : x;
            dist = Math.hypot(xCorner, yCorner);

            Vector velo = f.getVelocity();
            velo.setComponents(velo.getMagnitude(), velo.getTheta());

            double Vx = velo.getXComponent() * 0.0254;
            double Vy = velo.getYComponent() * 0.0254;

            TOF = ShooterMath.calculateTOF(ShooterMath.transformInMeters(ShooterMath.calculateVelocity(dist)),ShooterMath.calculateHoodAngle(dist));

            xCorner -= Vx * TOF* turretTOFCoef;
            yCorner -= Vy * TOF* turretTOFCoef;

            virtualDist = Math.hypot(xCorner, yCorner);

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
            turretRelative+= 180 + offset;
            double finalPos= turretRelative/360;
            if(finalPos>= 0.0475 && finalPos<=0.5) {
                turret1.setPosition(finalPos );
                turret2.setPosition(finalPos );

            }

        }
    }
    public static double getOffset(double headingRAD){
        double headingDEGREES=headingRAD*180/ Math.PI;
        double offset=0;
        if(headingDEGREES<90){
            offset=-3+(headingDEGREES/90)*(6-(-3));
        }
        else if(headingDEGREES<180){
            offset=6+((headingDEGREES-90)/90)*(3-6);
        }
        else if(headingDEGREES<270){
            offset=3+((headingDEGREES-180)/90)*(0-3);
        }
        else offset=0+((headingDEGREES-270)/90)*(-3-0);
        return offset;

    }
    public static double offset = 0;
    public static void update(){
        Pose robotPose= Robot.robotPose;
        //offset=getOffset(robotPose.getHeading());//nu merge
        if(Storage.status.equals(Storage.Status.STORAGEFULL) && !checkedStatus){
            turrStatus = TurretStatus.ACTIVE;
            checkedStatus = true;
        }
        if (robotPose.getX() != 0 && robotPose.getY() != 0) {

            x= robotPose.getX();
            y= robotPose.getY();
            heading= robotPose.getHeading();

            yCorner= FIELD_LENGTH- y;
            xCorner= Robot.a== Alliance.RED ? FIELD_LENGTH- x : x;

            dist = Math.hypot(xCorner, yCorner);

            double Vx = Robot.odo.getVelX(DistanceUnit.METER);
            double Vy = Robot.odo.getVelY(DistanceUnit.METER);
            double V = Math.hypot(Vx, Vy);

            //sotm = Shooter.readyForShoot && V > 1;

            TOF = ShooterMath.calculateTOF(ShooterMath.transformInMeters(ShooterMath.calculateVelocity(dist)),ShooterMath.calculateHoodAngle(dist));

            xCorner -= Vx * TOF * turretTOFCoef;
            yCorner -= Vy * TOF* turretTOFCoef;

            virtualDist = Math.hypot(xCorner, yCorner);

            if(turrStatus.equals(TurretStatus.ACTIVE)) {
                heading = Math.toDegrees(heading);
                xCorner -= goalCircleRasius;
                yCorner -= goalCircleRasius;
                fieldRelative = Math.atan2(yCorner, xCorner);
                fieldRelative = Math.toDegrees(fieldRelative);
                if (Robot.a == Alliance.BLUE)
                    fieldRelative = 180 - fieldRelative;
                robotRelative = normalize(fieldRelative - heading);
                turretRelative = normalize(robotRelative - 180);
                robotRelative = Robot.a == Alliance.RED ? robotRelative : -robotRelative;
                turretRelative += 180;

                turretRelative += offset;
                turretRelative = Range.clip(turretRelative, 0, 360);
                double finalPos = turretRelative / 360;
                if (Robot.playStyle == Robot.PlayStyle.CLOSE ? (Robot.a == Alliance.RED ? (finalPos >= 0.0475 && finalPos <= 0.5) : (finalPos > 0.5 && finalPos <= 0.95)) : (finalPos >= 0.0475 && finalPos <= 0.95)) {
                    turret1.setPosition(finalPos);
                    turret2.setPosition(finalPos);
                }
            } else setNeutralPosition(0.5);
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