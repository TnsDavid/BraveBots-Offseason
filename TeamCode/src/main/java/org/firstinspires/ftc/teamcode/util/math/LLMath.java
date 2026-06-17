package org.firstinspires.ftc.teamcode.util.math;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.Poses;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.LinkedList;
import java.util.List;

@Configurable
public class LLMath {
    public static final double ballHeight = 2.5,
            cameraMountAngle = 10,
            xOffset = 0,
            yOffset = 6.7,
            zOffset = 9.8,//8.85
            intakeWidth = 17,//inch
            robotLength = 17.36;

    public static Limelight3A ll;

    public static void init(HardwareMap hm){
        ll = hm.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(0);
        ll.setPollRateHz(100);
        ll.start();
    }

    public static LinkedList<Pose> results,RobotRelative,FieldRelative;

    public static void update(){
        results = new LinkedList<>();
       // RobotRelative= new LinkedList<>();
       // FieldRelative= new LinkedList<>();
        LLResult result = ll.getLatestResult();
        if(result != null && result.isValid()){
            List<LLResultTypes.DetectorResult> dtcRes = result.getDetectorResults();
            if(dtcRes != null) {
                for(LLResultTypes.DetectorResult dtc : dtcRes){
                    double tx = dtc.getTargetXDegrees();
                    double ty = dtc.getTargetYDegrees();
                    double xRR = getBallX(tx, ty);
                    double yRR = getBallY(tx, ty);
                    if(!Double.isNaN(tx) && !Double.isNaN(ty)){
                        results.add(new Pose(tx, ty));
                        //RobotRelative.add(new Pose(xRR,yRR));
                       // FieldRelative.add(getFieldRelativePose(new Pose(xRR,yRR)));
                    }
                }
            }
        }
    }

    public static double getBallY(double tx, double ty){
        double angleGoal = Math.toRadians(90 -cameraMountAngle +ty);
        double yp=(zOffset - ballHeight) * Math.tan(angleGoal);
        return yp*Math.cos(Math.toRadians(tx)) + yOffset;
    }

    public static double getBallX(double tx, double ty){
        double yp = (getBallY(tx, ty) - yOffset) / Math.cos(Math.toRadians(tx));
        return yp * Math.sin(Math.toRadians(tx)) + xOffset;
//        double hyp = Math.hypot(y, zOffset - ballHeight);
//        return Math.tan(Math.toRadians(tx)) * hyp + xOffset;
    }
    public static LinkedList<Pose> getRobotRelativePoses(){
        LinkedList<Pose> ballPosesRobotRelative = new LinkedList<>();
        for(Pose ballParams : results){
            double xComp = getBallX(ballParams.getX(), ballParams.getY());
            double yComp = getBallY(ballParams.getX(), ballParams.getY());
            Pose pose = new Pose(xComp, yComp);
            ballPosesRobotRelative.add(pose);
        }
        return ballPosesRobotRelative;
    }

    /*
    public static double getBallY(double tx, double ty) {
        // dz = cât de sus e camera față de centrul mingii
        double dz = zOffset - ballHeight;

        // Dacă cameraMountAngle = 15 înseamnă camera înclinată ÎN JOS cu 15 grade:
        // ty pozitiv = ținta apare mai sus în imagine => unghiul în jos scade
        double downAngle = cameraMountAngle - ty;

        // protecție contra tan(0), valori negative sau target imposibil
        if (downAngle <= 1.0) {
            return Double.NaN;
        }

        double forwardFromCamera = dz / Math.tan(Math.toRadians(downAngle));

        return forwardFromCamera + yOffset;
    }

    public static double getBallX(double tx, double ty) {
        double dz = zOffset - ballHeight;

        double downAngle = cameraMountAngle - ty;

        if (downAngle <= 1.0) {
            return Double.NaN;
        }

        double forwardFromCamera = dz / Math.tan(Math.toRadians(downAngle));
        double sidewaysFromCamera = forwardFromCamera * Math.tan(Math.toRadians(tx));

        return sidewaysFromCamera + xOffset;
    }

    public static LinkedList<Pose> getRobotRelativePoses() {
        LinkedList<Pose> ballPosesRobotRelative = new LinkedList<>();

        if (results == null) {
            return ballPosesRobotRelative;
        }

        for (Pose ballParams : results) {
            double xComp = getBallX(ballParams.getX(), ballParams.getY());
            double yComp = getBallY(ballParams.getX(), ballParams.getY());

            if (!Double.isNaN(xComp) && !Double.isNaN(yComp)
                    && Double.isFinite(xComp) && Double.isFinite(yComp)) {
                Pose pose = new Pose(xComp, yComp);
                ballPosesRobotRelative.add(pose);
            }
        }

        return ballPosesRobotRelative;
    }
    */
    public static Pose getFieldRelativePose(Pose p, Pose robotPose){
        Vector v = new Vector();
        v.setOrthogonalComponents(p.getY(), -p.getX());//sunt invers axele RL fata de FL pt bila <3
        v.rotateVector(robotPose.getHeading());

        return new Pose(
                MathStuff.metersToInches(robotPose.getX()) + v.getXComponent() ,
                MathStuff.metersToInches(robotPose.getY()) + v.getYComponent()
        );
    }
    public static Pose getBestRobotRelativePose(){
        LinkedList<Pose> ballPoses = getRobotRelativePoses();
        Pose bestPose = new Pose();
        double maxSeenInArea = 0;
        for(Pose ballPose : ballPoses){
            double seenInArea = 0;
            for(Pose ballPose2 : ballPoses)
                if(ballPose!=ballPose2 && Math.abs(ballPose.getX() - ballPose2.getX()) < intakeWidth / 2) seenInArea++;
            if(seenInArea > maxSeenInArea){
                maxSeenInArea = seenInArea;
                bestPose = ballPose;
            }
        }
        return bestPose;
    }

    public static void scan(Follower flw){
        update();
        Pose robotRelPose = getBestRobotRelativePose();
        Pose fieldRelPose = getFieldRelativePose(robotRelPose, flw.getPose());
        //Pose optimalPose = new Pose(Robot.a == Alliance.RED ? fieldRelPose.getX() - robotLength / 2 : fieldRelPose.getX() + robotLength, fieldRelPose.getY());
        Pose optimalPose = new Pose(Robot.a == Alliance.RED ? fieldRelPose.getX() - robotLength / 2 : fieldRelPose.getX() + robotLength/2, fieldRelPose.getY());
        Path path = new Path(new BezierLine(flw.getPose(), optimalPose));
        path.setConstantHeadingInterpolation(Robot.a == Alliance.RED ? Math.toRadians(0) : Math.toRadians(180));
        flw.followPath(path);
    }
    public static Pose optimalPose=new Pose();
    public static void scanPose (Follower flw){
        update();
        optimalPose=new Pose(Poses.FarPoses.HPCollectPose.getX(),Poses.FarPoses.HPCollectPose.getY(),Poses.FarPoses.HPCollectPose.getHeading());
        Pose robotRelPose = getBestRobotRelativePose();
        Pose fieldRelPose = getFieldRelativePose(robotRelPose, flw.getPose());
        optimalPose = new Pose(Robot.a == Alliance.RED ? fieldRelPose.getX() - robotLength / 2 : fieldRelPose.getX() + robotLength/2, fieldRelPose.getY(),Robot.a == Alliance.RED ? Math.toRadians(0) : Math.toRadians(180));
    }

    public static Pose test(Pose robotPose){
        update();
        return getFieldRelativePose(getBestRobotRelativePose(),robotPose);
    }
}
/*
public static Pose getBallRobotRelative(double txDeg, double tyDeg) {
    double dz = zOffset - ballHeight;

    // Camera înclinată în jos față de orizontală.
    double downAngleDeg = cameraMountAngle - tyDeg;

    if (downAngleDeg <= 1.0) return null;

    double forward = dz / Math.tan(Math.toRadians(downAngleDeg));
    if (!Double.isFinite(forward) || forward < 0) return null;

    double right = forward * Math.tan(Math.toRadians(txDeg));

    return new Pose(right + xOffset, forward + yOffset);
}

public static LinkedList<Pose> getRobotRelativePoses() {
    LinkedList<Pose> ballPosesRobotRelative = new LinkedList<>();

    if (results == null) return ballPosesRobotRelative;

    for (Pose ballParams : results) {
        Pose pose = getBallRobotRelative(ballParams.getX(), ballParams.getY());
        if (pose != null) {
            ballPosesRobotRelative.add(pose);
        }
    }

    return ballPosesRobotRelative;
}

public static Pose getFieldRelativePose(Pose robotRel, Pose robotFieldPose) {
    double right = robotRel.getX();
    double forward = robotRel.getY();
    double h = robotFieldPose.getHeading();

    double fieldDx = forward * Math.cos(h) + right * Math.sin(h);
    double fieldDy = forward * Math.sin(h) - right * Math.cos(h);

    return new Pose(
            robotFieldPose.getX() + fieldDx,
            robotFieldPose.getY() + fieldDy
    );
}
 */
