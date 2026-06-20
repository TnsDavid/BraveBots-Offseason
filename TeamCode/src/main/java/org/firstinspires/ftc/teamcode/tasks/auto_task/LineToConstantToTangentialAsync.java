package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;


/// sa fie doar pt primu path
public class LineToConstantToTangentialAsync implements Task {
    Follower f;
    Pose p;

    boolean started;
    double tvalue;
    public LineToConstantToTangentialAsync(Follower f, Pose p , double tvalue){
        this.f= f;
        this.p= p;
        started= false;
        this.tvalue = tvalue;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierLine(f.getPose() , p));
            path.setHeadingInterpolation(HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(
                            0,
                            tvalue,
                            HeadingInterpolator.constant(Math.toRadians(270))
                    ) ,
                    new HeadingInterpolator.PiecewiseNode(
                            tvalue ,
                            1 ,
                            HeadingInterpolator.tangent
                    )
            ));
            f.followPath(path);
            started= true;
        }
        return !f.isBusy();
    }
}
