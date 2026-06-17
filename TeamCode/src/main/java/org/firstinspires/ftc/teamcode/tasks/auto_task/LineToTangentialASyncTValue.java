package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class LineToTangentialASyncTValue implements Task {
    Follower f;
    Pose p;
    public boolean started;
    public double tValue;

    public LineToTangentialASyncTValue(Follower f, Pose p, double tValue) {
        this.f = f;
        this.p = p;
        this.tValue = tValue;
        started = false;
    }

    @Override
    public boolean Run() {
        if( !started ) {
            Path path = new Path(new BezierLine(f.getPose(), p));
            path.setHeadingInterpolation(HeadingInterpolator.tangent);
            f.followPath(path);
            started = true;
        }
        return f.getCurrentTValue() >= tValue;
    }
}
