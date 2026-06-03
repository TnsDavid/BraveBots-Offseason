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
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.LinkedList;
import java.util.List;

@Configurable
public class LLMath {
    public static final double ballHeight = 2.5,
            cameraMountAngle = 5,
            xOffset = 0,
            yOffset = 8,
            zOffset = 0,
            intakeWidth = 0; //inch

    public static Limelight3A ll;

    public static void init(HardwareMap hm){
        ll = hm.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(0);
        ll.setPollRateHz(100);
        ll.start();
    }

    public static LinkedList<Pose> results;

    public static void update(){
        results = new LinkedList<>();
        LLResult result = ll.getLatestResult();
        if(result != null && result.isValid()){
            List<LLResultTypes.DetectorResult> dtcRes = result.getDetectorResults();
            if(dtcRes != null) {
                for(LLResultTypes.DetectorResult dtc : dtcRes){
                    double tx = dtc.getTargetXDegrees();
                    double ty = dtc.getTargetYDegrees();
                    if(!Double.isNaN(tx) && !Double.isNaN(ty)){
                        results.add(new Pose(tx, ty));
                    }
                }
            }
        }
    }

    public static double getBallY(double ty){
        double angleGoal = Math.toRadians(90 + cameraMountAngle + ty);
        return (zOffset - ballHeight) * Math.tan(angleGoal) + yOffset;
    }

    public static double getBallX(double tx, double ty){
        double y = getBallY(ty);
        double hyp = Math.hypot(y, zOffset - ballHeight);
        return Math.tan(Math.toRadians(tx)) * hyp + xOffset;
    }
    public static LinkedList<Pose> getRobotRelativePoses(){
        LinkedList<Pose> ballPoseRobotRelative = new LinkedList<>();
        for(Pose ballParams : results){
            double xComp = getBallX(ballParams.getX(), ballParams.getY());
            double yComp = getBallY(ballParams.getY());
            Pose pose = new Pose(xComp, yComp);
            ballPoseRobotRelative.add(pose);
        }
        return ballPoseRobotRelative;
    }

    public static Pose getFieldRelativePose(Pose p){
        Vector v = new Vector();
        v.setOrthogonalComponents(p.getX(), p.getY());
        v.rotateVector(Robot.robotPose.getHeading());

        return new Pose(
                Robot.robotPose.getX() + v.getXComponent(),
                Robot.robotPose.getY() + v.getYComponent()
        );
    }
    public static Pose getBestRobotRelativePose(){
        update();
        LinkedList<Pose> ballPoses = getRobotRelativePoses();
        Pose bestPose = new Pose();
        double maxSeenInArea = 0;
        for(Pose ballPose : ballPoses){
            double seenInArea = 0;
            for(Pose ballPose2 : ballPoses)
                if(Math.abs(ballPose.getX() - ballPose2.getX()) > intakeWidth / 2) seenInArea++;
            if(seenInArea > maxSeenInArea){
                maxSeenInArea = seenInArea;
                bestPose = ballPose;
            }
        }
        return bestPose;
    }

    public static void scan(Follower flw){
        Pose robotRelPose = getBestRobotRelativePose();
        Pose fieldRelPose = getFieldRelativePose(robotRelPose);

        Path path = new Path(new BezierLine(flw.getPose(), fieldRelPose));
        path.setConstantHeadingInterpolation(Robot.a == Alliance.RED ? Math.toRadians(0) : Math.toRadians(180));
        flw.followPath(path);
    }
}
