package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class LineToConstantSync implements Task {
    Follower f;
    Pose p;
    boolean started;
    double heading;
    public LineToConstantSync(Follower f, Pose p, double heading){
        this.f= f;
        this.p= p;
        this.heading= heading;
        started= false;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierLine(f.getPose(), p));
            path.setHeadingInterpolation(HeadingInterpolator.constant(heading));
            f.followPath(path);
            started= true;
        }
        return true;
    }

}
