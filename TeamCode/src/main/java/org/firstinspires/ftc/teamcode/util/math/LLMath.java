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

import org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses.Poses;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;

import java.util.LinkedList;
import java.util.List;

@Configurable
public class LLMath {
    public static final double ballHeight = 2.5,
            cameraMountAngle = 10,
            fieldLength = 144,
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
        v.setOrthogonalComponents(p.getY(), -p.getX());
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
        for(Pose ballPose : ballPoses) {
            double seenInArea = 0;
            for (Pose ballPose2 : ballPoses) {
                boolean conditie = Math.abs(ballPose.getX() - ballPose2.getX()) < intakeWidth / 2;
                if (conditie) seenInArea++;
            }
            if (seenInArea > maxSeenInArea) {
                maxSeenInArea = seenInArea;
                bestPose = ballPose;
                bestPose=new Pose(bestPose.getX(), bestPose.getY());
            }
            else if(seenInArea==maxSeenInArea){
                if(ballPose.getX()>bestPose.getX()){
                    bestPose = ballPose;
                    bestPose=new Pose(bestPose.getX(), bestPose.getY());
                }
            }
        }
        return bestPose;
    }


    public static  double y = 0;
    public static boolean scan(Follower flw){
        optimalPose=Robot.alliance == Alliance.RED ? Poses.FarPoses.HPCollectPose : Poses.FarPoses.HPCollectPose.mirror(140);
        update();
        if(!results.isEmpty())
            for(int i=0;i<2;i++) {
                update();
                Pose robotRelPose = getBestRobotRelativePose();
                Pose fieldRelPose = getFieldRelativePose(robotRelPose, flw.getPose());
                optimalPose = new Pose(130.2, fieldRelPose.getY());
                if(Robot.alliance == Alliance.BLUE) optimalPose = new Pose( 11.7, optimalPose.mirror().getY());
            }
        Path path = new Path(new BezierLine(flw.getPose(), optimalPose));
        path.setConstantHeadingInterpolation(Robot.alliance == Alliance.RED ? Math.toRadians(0) : Math.toRadians(180));
        flw.followPath(path);
        return true;
    }

    public static void test(Pose robotPose){
        update();
        optimalPose = getFieldRelativePose(getBestRobotRelativePose(),robotPose);
    }
}