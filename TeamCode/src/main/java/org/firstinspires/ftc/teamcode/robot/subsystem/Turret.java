package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;

@Configurable
public class Turret {

    static double NEUTRAL_POSITION= .5,turretRange=351;
    public static double x, y, heading, xCorner, yCorner, dist, fieldRelative, robotRelative, turretRelative, TOF, virtualDist;
    public static final double FIELD_LENGTH= 3.65, launchTime= .2, goalCircleRadius = .18;
    //y=-0.0239556x^{2}+0.242631x+0.322645
    public static final double at = -0.0275109, bt = 0.253576, ct = 0.273474;
    public static double tofCoeff = 1.15, shooterTOFCoef = 1.05;
    public static BetterServoEx turret1, turret2;
    public static boolean reseted = false;
    public static boolean sotm = false;
    public static boolean checkedStatus = false;

    public static LowPassFilter ChassisLpf;

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
            xCorner= Robot.alliance == Alliance.RED ? FIELD_LENGTH- x : -x;
            dist = Math.hypot(xCorner, yCorner);

            Vector velo = f.getVelocity();
            velo.setComponents(velo.getMagnitude(), velo.getTheta());
            velo.setMagnitude(ChassisLpf.update(velo.getMagnitude()));

            double Vx = velo.getXComponent() * 0.0254;
            double Vy = velo.getYComponent() * 0.0254;

            TOF = tofCoeff * ShooterMath.calculateTOF(ShooterMath.transformInMeters(ShooterMath.calculateVelocity(dist)),ShooterMath.calculateHoodAngle(dist)) * tofCoeff;

            xCorner -= Vx * TOF;
            yCorner -= Vy * TOF;

            virtualDist = Math.hypot(xCorner, yCorner);
            xCorner += Robot.alliance == Alliance.RED ?-goalCircleRadius : goalCircleRadius;
            yCorner -= goalCircleRadius;


            heading = Math.toDegrees(heading);

            fieldRelative = Math.atan2(yCorner, xCorner);
            fieldRelative = Math.toDegrees(fieldRelative);
            robotRelative = normalize(fieldRelative - heading);
            turretRelative = normalize(robotRelative - 180);
            turretRelative += 180;

            turretRelative += offset;
            turretRelative = Range.clip(turretRelative, 0, 360);
            double finalPos = 0.5 + (turretRelative-180) / turretRange;
            if(Robot.alliance ==Alliance.RED){
                if(finalPos>= 0.0475 && finalPos<=0.5){
                    turret1.setPosition(finalPos);
                    turret2.setPosition(finalPos);
                }
            }
            else{
                if(finalPos>= 0.5 && finalPos<=0.95){
                    turret1.setPosition(finalPos);
                    turret2.setPosition(finalPos);
                }
            }

        }
    }
    public static double offset = 0;
    public static double alpha;
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

            yCorner= FIELD_LENGTH- y - goalCircleRadius;
            xCorner= Robot.alliance == Alliance.RED ? FIELD_LENGTH- x - goalCircleRadius : -x + goalCircleRadius;

//            dist = Math.hypot(xCorner, yCorner);

            double Vx = Robot.odo.getVelX(DistanceUnit.METER);
            double Vy = Robot.odo.getVelY(DistanceUnit.METER);
            Vector velocity = new Vector();
            velocity.setOrthogonalComponents(Vx , Vy);
            velocity.setMagnitude(ChassisLpf.update(velocity.getMagnitude()));

            sotm = Shooter.readyForShoot && velocity.getMagnitude() > 1;

            TOF = tofCoeff *  ShooterMath.calculateTOF(ShooterMath.transformInMeters(ShooterMath.calculateVelocity(dist)),ShooterMath.calculateHoodAngle(dist));

            xCorner -= velocity.getXComponent() * TOF;
            yCorner -= velocity.getYComponent() * TOF;

            virtualDist = Math.hypot(xCorner, yCorner);

            if(turrStatus.equals(TurretStatus.ACTIVE)) {
                Vector turrVector = new Vector();
                turrVector.setOrthogonalComponents(xCorner, yCorner);
                turrVector.rotateVector(-heading);// posibil -heading, trebuie verificat
                alpha = Math.toDegrees(turrVector.getTheta());
                alpha = Range.clip(alpha, 0, 360);

                double finalPos = 0.5 + (alpha - 180) / turretRange;
                if (finalPos >= 0.0475 && finalPos <= 0.95) {
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