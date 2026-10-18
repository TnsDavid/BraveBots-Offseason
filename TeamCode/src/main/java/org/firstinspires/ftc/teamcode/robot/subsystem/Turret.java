package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.util.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;

@Configurable
public class Turret {

    public static double turretRange=351;
    public static double x, y, heading, xCorner, yCorner, dist, TOF, virtualDist;
    public static final double FIELD_LENGTH= 3.65, goalCircleRadius = .18;
    public static double tofCoeff = 1.15;
    public static BetterServoEx turret1, turret2;
    public static double xDelta, yDelta;
    public static boolean reseted = false;
    public static boolean sotm = false;
    public static boolean checkedStatus = false;

    public static LowPassFilter ChassisLpf;

    public enum TurretStatus {
        INACTIVE, ACTIVE
    }

    public static TurretStatus turrStatus = TurretStatus.INACTIVE;

    public static void toggleStatus(){
        turrStatus = turrStatus.equals(TurretStatus.ACTIVE) ? TurretStatus.INACTIVE : TurretStatus.ACTIVE;
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

            yCorner= FIELD_LENGTH- y - goalCircleRadius;
            xCorner= Robot.alliance == Alliance.RED ? FIELD_LENGTH- x - goalCircleRadius : -x + goalCircleRadius;

            double Vx = Odo.odo.getVelX(DistanceUnit.METER);
            double Vy = Odo.odo.getVelY(DistanceUnit.METER);
            Vector velocity = new Vector();
            velocity.setOrthogonalComponents(Vx , Vy);
            velocity.setMagnitude(ChassisLpf.update(velocity.getMagnitude()));

            sotm = Shooter.readyForShoot && velocity.getMagnitude() > 1;

            TOF = tofCoeff *  ShooterMath.calculateTOF(ShooterMath.transformInMeters(ShooterMath.calculateVelocityFromDistance(dist)),ShooterMath.calculateHoodAngleFromDistance(dist));

            xCorner -= velocity.getXComponent() * TOF;
            yCorner -= velocity.getYComponent() * TOF;

            virtualDist = Math.hypot(xCorner, yCorner);

            if(turrStatus.equals(TurretStatus.ACTIVE)) {
                Vector turrVector = new Vector();
                turrVector.setOrthogonalComponents(xCorner, yCorner);
                turrVector.rotateVector(-heading);
                alpha = Math.toDegrees(turrVector.getTheta()) + offset;
                alpha = Range.clip(alpha, 0, 360);

                double finalPos = 0.5 + (alpha - 180) / turretRange;
                if (finalPos >= 0.0475 && finalPos <= 0.95) {
                    turret1.setPosition(finalPos);
                    turret2.setPosition(finalPos);
                }
            } else setNeutralPosition(0.5);
        }
    }
    public static double offset = 2, addedOffset = 0 , coef = 4.5 ;
    public static double xComp, yComp;
    public static final double hiveLength = 20, hiveWidth = 14;
    public static double alpha, offsetCenter = 1.5, adjustableOffset = 4.5;
    public static void update(){
        Pose robotPose= Robot.robotPose;
        if (robotPose.getX() != 0 && robotPose.getY() != 0) {

            double robotX = robotPose.getX() * 39.3701;
            double robotY = robotPose.getY() * 39.3701;
            double robotHeading = robotPose.getHeading();

            boolean switchedHive = robotY > 72;
            double hiveX = (Robot.alliance == Alliance.RED) ? 60 : 84;
            double hiveY = (robotY < 72) ? 60 : 84;
            Pose hivePose = new Pose(hiveX, hiveY);

            xDelta = hivePose.getX() - robotX;
            double xOffset = (xDelta / hivePose.getX()) * (hiveLength / 2);
            double yOffset = switchedHive ? (hivePose.getY() / robotY) * (hiveWidth / 2) : -(robotY / hivePose.getY()) * (hiveWidth / 2) ;
            if(Math.abs(xOffset) > hiveLength / 2 || Math.abs(yOffset) > hiveWidth / 2) /*limit logic*/ ;
            
            double Vx = Odo.odo.getVelX(DistanceUnit.METER);
            double Vy = Odo.odo.getVelY(DistanceUnit.METER);
            Vy *= switchedHive ? -1 : 1;
            double TOF = 1;
            xOffset -= Vx * TOF;
            yOffset -= Vy * TOF;

            hivePose = new Pose(hivePose.getX() + xOffset, hivePose.getY() + yOffset);
            xDelta = hivePose.getX() - robotX;
            yDelta = hivePose.getY() - robotY;

            virtualDist = Math.hypot(xDelta, yDelta);

            if(turrStatus.equals(TurretStatus.ACTIVE)) {
                Vector offsetVector = new Vector();
                offsetVector.setOrthogonalComponents(-offsetCenter, 0);
                offsetVector.rotateVector(robotHeading);

                xDelta -= offsetVector.getXComponent();
                yDelta -= offsetVector.getYComponent();

                Vector turrVector = new Vector();
                turrVector.setOrthogonalComponents(xDelta, yDelta);
                turrVector.rotateVector(-robotHeading);

                alpha = Math.toDegrees(turrVector.getTheta()) + offset;
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