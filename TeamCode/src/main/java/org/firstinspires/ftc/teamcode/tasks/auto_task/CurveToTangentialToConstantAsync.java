package org.firstinspires.ftc.teamcode.tasks.auto_task;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class CurveToTangentialToConstantAsync implements Task {
    Follower f;
    Pose p , controlPoint;

    boolean started;
    double tvalue;
    public CurveToTangentialToConstantAsync(Follower f, Pose controlPoint, Pose p , double tvalue){
        this.f= f;
        this.p= p;
        this.controlPoint = controlPoint;
        started= false;
        this.tvalue = tvalue;
    }
    @Override
    public boolean Run() {
        if(!started){
            Path path= new Path(new BezierCurve(f.getPose() , controlPoint, p));
            path.setHeadingInterpolation(HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(
                            0,
                            tvalue,
                            HeadingInterpolator.tangent
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
        return !f.isBusy();
    }
}
