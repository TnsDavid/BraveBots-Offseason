package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.auto.ExecutionMode;

import java.util.List;

public interface BetterTask {
    static BetterTaskBuilder createTask(){
        return new BetterTaskBuilder();
    }
    static BetterTask createSimpleTask(Runnable execute, TaskType type){
        return BetterTask.createTask()
                .setTaskExecute(execute)
                .setTaskType(type);
    }
    static BetterTask createInfinite(Runnable infinite){
        return BetterTask.createTask()
                .setTaskExecute(infinite)
                .setTaskType(TaskType.INFINITE);
    }
    static BetterTask createMoveCommand(Follower f, ExecutionMode mode, List<Pose> poses){
        return BetterTask.createTask()
                .setTaskExecute(() -> {
                    if(!poses.isEmpty()) {
                        Path path;
                        if (poses.size() > 2) path = new Path(new BezierCurve(poses));
                        else path = new Path(new BezierLine(f.getPose(), poses.get(0)));
                        path.setHeadingInterpolation(HeadingInterpolator.tangent);
                        f.followPath(path);
                    }
                })
                .setExecuteType(ExecuteType.ONCE)
                .setTaskEnd(interrupted -> {
                    if(interrupted) f.breakFollowing();
                })
                .setTaskType(TaskType.SEQUENTIAL)
                .setTaskJustDone(mode == ExecutionMode.ASYNC ? f::isBusy : () -> true);
    }

    static BetterTask createMoveCommand(Follower f, ExecutionMode mode, double heading, List<Pose> poses){
        return BetterTask.createTask()
                .setTaskExecute(() -> {
                    if(!poses.isEmpty()) {
                        Path path;
                        if (poses.size() > 2) path = new Path(new BezierCurve(poses));
                        else path = new Path(new BezierLine(f.getPose(), poses.get(0)));
                        path.setHeadingInterpolation(HeadingInterpolator.constant(Math.toRadians(heading)));
                        f.followPath(path);
                    }
                })
                .setExecuteType(ExecuteType.ONCE)
                .setTaskEnd(interrupted -> {
                    if(interrupted) f.breakFollowing();
                })
                .setTaskType(TaskType.SEQUENTIAL)
                .setTaskJustDone(mode == ExecutionMode.ASYNC ? f::isBusy : () -> true);
    }

    static BetterTask createMoveCommand(Follower f, ExecutionMode mode, double startHeading, double endHeading, List<Pose> poses){
        return BetterTask.createTask()
                .setTaskExecute(() -> {
                    if(!poses.isEmpty()) {
                        Path path;
                        if (poses.size() > 2) path = new Path(new BezierCurve(poses));
                        else path = new Path(new BezierLine(f.getPose(), poses.get(0)));
                        path.setHeadingInterpolation(HeadingInterpolator.linear(startHeading, endHeading));
                        f.followPath(path);
                    }
                })
                .setExecuteType(ExecuteType.ONCE)
                .setTaskEnd(interrupted -> {
                    if(interrupted) f.breakFollowing();
                })
                .setTaskType(TaskType.SEQUENTIAL)
                .setTaskJustDone(mode == ExecutionMode.ASYNC ? f::isBusy : () -> true);
    }


    boolean done();
    void start();
    void end(boolean interrupted);
    void execute();
    ExecuteType getExecuteType();
    TaskType getType();
}