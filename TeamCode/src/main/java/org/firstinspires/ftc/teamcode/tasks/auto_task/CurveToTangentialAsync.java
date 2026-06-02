package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class CurveToTangentialAsync implements Task {

    Follower f;
    Pose p, controlPoint;

    boolean started;
    public CurveToTangentialAsync(Follower f, Pose controlPoint , Pose p){
        this.f= f;
        this.p= p;
        this.controlPoint= controlPoint;
        started= false;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierCurve(f.getPose(), controlPoint, p));
            path.setHeadingInterpolation(HeadingInterpolator.tangent);
            f.followPath(path);
            started= true;
        }
        return !f.isBusy();
    }
}
