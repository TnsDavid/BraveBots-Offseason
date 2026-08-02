package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class LineToTangentialToConstantSync implements Task {
    Follower f;
    Pose p;

    boolean started, reverse;
    double tvalue;
    public LineToTangentialToConstantSync(Follower f, Pose p , double tvalue, boolean reverse){
        this.f= f;
        this.p= p;
        started= false;
        this.tvalue = tvalue;
        this.reverse = reverse;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierLine(f.getPose() , p));
            path.setHeadingInterpolation(HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(
                            0,
                            tvalue,
                            !reverse ? HeadingInterpolator.tangent : HeadingInterpolator.tangent.reverse()
                    ) ,
                    new HeadingInterpolator.PiecewiseNode(
                            tvalue ,
                            1 ,
                            HeadingInterpolator.constant(p.getHeading())
                    )
            ));
            f.followPath(path);
            started= true;
        }
        return true;
    }
}
