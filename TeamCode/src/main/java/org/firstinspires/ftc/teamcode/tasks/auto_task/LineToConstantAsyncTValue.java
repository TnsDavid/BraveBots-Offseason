package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class LineToConstantAsyncTValue implements Task {

    Follower f;
    Pose p;
    boolean started;
    double heading;
    double tValue;
    public  LineToConstantAsyncTValue(Follower f, Pose p, double heading,double tValue){
        this.f= f;
        this.p= p;
        started= false;
        this.heading= heading;
        this.tValue = tValue;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierLine(f.getPose(), p));
            path.setHeadingInterpolation(HeadingInterpolator.constant(heading));
            f.followPath(path);
            started= true;
        }
        return f.getCurrentTValue()>=tValue;
    }
}
