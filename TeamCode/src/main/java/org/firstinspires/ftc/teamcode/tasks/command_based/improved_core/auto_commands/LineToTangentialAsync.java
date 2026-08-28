package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.auto_commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.ExecuteType;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.TaskType;

public class LineToTangentialAsync {
    public static BetterTask command;
    public static void createCommand(Follower f, Pose p){
        command = BetterTask.createTask()
                .setTaskExecute(() -> {
                    Path path= new Path(new BezierLine(f.getPose(), p));
                    path.setHeadingInterpolation(HeadingInterpolator.tangent);
                    f.followPath(path);
                })
                .setExecuteType(ExecuteType.ONCE)
                .setTaskEnd(interrupted -> {
                    if(interrupted) f.breakFollowing();
                })
                .setTaskType(TaskType.SEQUENTIAL)
                .setTaskJustDone(f::isBusy);
    }
    public static BetterTask getCommand(){
        return command;
    }
}
