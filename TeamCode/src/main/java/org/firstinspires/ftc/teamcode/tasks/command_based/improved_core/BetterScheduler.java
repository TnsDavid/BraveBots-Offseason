package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.auto_commands.LineToTangentialAsync;

import java.util.Iterator;
import java.util.LinkedList;

public class BetterScheduler {
    public LinkedList<BetterTask> list;
    public LinkedList<BetterTask> parallelList;
    public LinkedList<BetterTask> infiniteList;

    public Follower follower;
    public boolean executeTaskCondition = false;
    public BetterScheduler(){
        list = new LinkedList<>();
        parallelList = new LinkedList<>();
        infiniteList = new LinkedList<>();
    }

    public BetterScheduler(Follower follower){
        list = new LinkedList<>();
        parallelList = new LinkedList<>();
        infiniteList = new LinkedList<>();
        this.follower = follower;
    }

    public BetterScheduler schedule(BetterTask task){
        if(task.getType().equals(TaskType.SEQUENTIAL)) list.addLast(task);
        else if(task.getType().equals(TaskType.PARALLEL)) parallelList.addLast(task);
        else if(task.getType().equals(TaskType.PARALLEL)) infiniteList.addLast(task);
        return this;
    }

    public BetterScheduler schedule(BetterTask... tasks){
        for(BetterTask task : tasks){
            if(task.getType().equals(TaskType.SEQUENTIAL)) list.addLast(task);
            else if(task.getType().equals(TaskType.PARALLEL)) parallelList.addLast(task);
            else if(task.getType().equals(TaskType.PARALLEL)) infiniteList.addLast(task);
        }
        return this;
    }

    public BetterScheduler now(BetterTask task){
        if(task.getType().equals(TaskType.SEQUENTIAL)){
            list.getFirst().end(true);
            clearSequential();
            list.addLast(task);
        }
        else if(task.getType().equals(TaskType.PARALLEL)){
            parallelList.getFirst().end(true);
            clearParallel();
            parallelList.addLast(task);
        }
        return this;
    }

    public BetterScheduler clearSequential(){
        list.clear();
        return this;
    }

    public BetterScheduler clearParallel(){
        parallelList.clear();
        return this;
    }

    public BetterScheduler clearInfinite(){
        infiniteList.clear();
        return this;
    }

    public BetterScheduler lineToTangentialAsync(Pose pose){
        if(follower != null) {
            LineToTangentialAsync.createCommand(follower, pose);
            schedule(LineToTangentialAsync.getCommand());
        }
        return this;
    }

    public void update(){
        if(!list.isEmpty()){
            BetterTask currentTask = list.getFirst();
            if(!executeTaskCondition) currentTask.execute();
            if(currentTask.getExecuteType() == ExecuteType.ONCE) executeTaskCondition = true;
            if(currentTask.done()){
                currentTask.end(false);
                executeTaskCondition = false;
                list.removeFirst();
            }
        }
        if(!parallelList.isEmpty()){
            Iterator<BetterTask> it = parallelList.iterator();
            while(it.hasNext()){
                BetterTask task = it.next();
                task.execute();
                task.end(false);
                it.remove();
            }
        }
        if(!infiniteList.isEmpty()){
            for (BetterTask infiniteTask : infiniteList) {
                infiniteTask.execute();
            }
        }
    }
}
