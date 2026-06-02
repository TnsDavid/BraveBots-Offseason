package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class CurveToConstantAsyncReversed implements Task {

    Follower f;
    Pose p, controlPoint;

    boolean started;
    double heading;
    public CurveToConstantAsyncReversed(Follower f, Pose controlPoint , Pose p, double heading){
        this.f= f;
        this.p= p;
        this.controlPoint= controlPoint;
        started= false;
        this.heading= heading;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierCurve(f.getPose(), controlPoint, p));
            path.setHeadingInterpolation(HeadingInterpolator.constant(heading));
            path.reverseHeadingInterpolation();
            f.followPath(path);
            started= true;
        }
        return !f.isBusy();
    }
}
