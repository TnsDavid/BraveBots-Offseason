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
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static Pose optimalPose = new Pose();
    public static double ballVelo = 0;

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


    public static Pose getFieldRelativePose(Pose p, Pose robotPose){
        Vector v = new Vector();
        v.setOrthogonalComponents(p.getY(), -p.getX());//sunt invers axele RL fata de FL pt bila <3
        v.rotateVector(robotPose.getHeading());

        return new Pose(
                robotPose.getX() + v.getXComponent() ,
                robotPose.getY() + v.getYComponent()
        );
    }
    public static Pose getBestRobotRelativePose(){
        LinkedList<Pose> ballPoses = getRobotRelativePoses();
        Pose bestPose = new Pose();
        double maxSeenInArea = 0;
        double FurthestY=0;
        for(Pose ballPose : ballPoses) {
            double seenInArea = 0;
            double BestYinArea=ballPose.getY();
            for (Pose ballPose2 : ballPoses) {
                BestYinArea = Math.max(BestYinArea, ballPose2.getY());
                boolean conditie = Math.abs(ballPose.getX() - ballPose2.getX()) < intakeWidth / 2;
                if (conditie) seenInArea++;
            }
                if (seenInArea > maxSeenInArea) {

                    maxSeenInArea = seenInArea;
                    bestPose = ballPose;
                    FurthestY=BestYinArea;
                    bestPose=new Pose(bestPose.getX(),FurthestY);
                }
                else if(seenInArea==maxSeenInArea){
                    if(ballPose.getX()>bestPose.getX()){
                        bestPose = ballPose;
                        FurthestY=BestYinArea;
                        bestPose=new Pose(bestPose.getX(),FurthestY);

                    }
                }
        }
        return bestPose;
    }

    public static boolean scan(Follower flw){
        optimalPose=Poses.FarPoses.HPCollectPose;
        update();
        if(!results.isEmpty())
            for(int i=0;i<2;i++) {
                update();
                Pose robotRelPose = getBestRobotRelativePose();
                Pose fieldRelPose = getFieldRelativePose(robotRelPose, flw.getPose());
                optimalPose = new Pose((Robot.a == Alliance.RED ? (fieldRelPose.getX() - robotLength / 2) : (fieldRelPose.getX() + robotLength/2)), (fieldRelPose.getY() - 5 >= 3) ? (fieldRelPose.getY() - 5) : fieldRelPose.getY() , flw.getHeading() + Math.toRadians(5));
            }
        Path path = new Path(new BezierLine(flw.getPose(), optimalPose));
        path.setConstantHeadingInterpolation(Robot.a == Alliance.RED ? Math.toRadians(0) : Math.toRadians(180));
        flw.followPath(path);
        return true;
    }

    public static void test(Pose robotPose){
        update();
        optimalPose = getFieldRelativePose(getBestRobotRelativePose(),robotPose);
    }
}