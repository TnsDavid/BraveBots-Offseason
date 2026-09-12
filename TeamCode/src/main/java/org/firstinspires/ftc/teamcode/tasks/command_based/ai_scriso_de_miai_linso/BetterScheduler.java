package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.auto.ExecutionMode;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.useful_commands.WaitUntil;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;

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

    public void addTask(BetterTask task){
        switch(task.getType()){
            case SEQUENTIAL:
                list.addLast(task);
                break;
            case PARALLEL:
                parallelList.addLast(task);
                break;
            case INFINITE:
                infiniteList.addLast(task);
                break;
        }
    }

    public BetterScheduler schedule(BetterTask task){
        addTask(task);
        return this;
    }

    public BetterScheduler schedule(LinkedList<BetterTask> tasks){
        for(BetterTask task : tasks){
            addTask(task);
        }
        return this;
    }

    public BetterScheduler schedule(BetterTask... tasks){
        for(BetterTask task : tasks){
            addTask(task);
        }
        return this;
    }

    public BetterScheduler now(BetterTask task){
        switch(task.getType()){
            case SEQUENTIAL:
                if(!list.isEmpty())
                    list.getLast().end(true);
                clearSequential();
                list.addLast(task);
                executeTaskCondition = false;
            case PARALLEL:
                if(!parallelList.isEmpty())
                    parallelList.getLast().end(true);
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

    public BetterScheduler moveTo(ExecutionMode executionMode, Pose... poses){
        if(follower != null){
            LinkedList<Pose> controlPoints = new LinkedList<>();
            controlPoints.addLast(follower.getPose());
            for(Pose pose : poses) controlPoints.addLast(pose);
            schedule(BetterTask.createMoveCommand(follower, executionMode, controlPoints));
        }
        return this;
    }

    public BetterScheduler moveTo(ExecutionMode executionMode, double heading, Pose... poses){
        if(follower != null){
            LinkedList<Pose> controlPoints = new LinkedList<>();
            controlPoints.addLast(follower.getPose());
            for(Pose pose : poses) controlPoints.addLast(pose);
            schedule(BetterTask.createMoveCommand(follower, executionMode, heading, controlPoints));
        }
        return this;
    }

    public BetterScheduler moveTo(ExecutionMode executionMode, double startHeading, double endHeading, Pose... poses){
        if(follower != null){
            LinkedList<Pose> controlPoints = new LinkedList<>();
            controlPoints.addLast(follower.getPose());
            for(Pose pose : poses) controlPoints.addLast(pose);
            schedule(BetterTask.createMoveCommand(follower, executionMode, startHeading, endHeading, controlPoints));
        }
        return this;
    }

    public BetterScheduler waitUntil(BooleanSupplier condition){
        WaitUntil.createCommand(condition);
        schedule(WaitUntil.getCommand());
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
                if(task.done()) {
                    task.end(false);
                    it.remove();
                }
            }
        }
        if(!infiniteList.isEmpty()){
            for (BetterTask infiniteTask : infiniteList) {
                infiniteTask.execute();
            }
        }
    }
}
