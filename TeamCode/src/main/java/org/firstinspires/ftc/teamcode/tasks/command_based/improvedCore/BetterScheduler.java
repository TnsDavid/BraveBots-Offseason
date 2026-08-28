package org.firstinspires.ftc.teamcode.tasks.command_based.improvedCore;

import java.util.Iterator;
import java.util.LinkedList;

public class BetterScheduler {
    public static LinkedList<BetterTask> list = new LinkedList<>();
    public static LinkedList<BetterTask> parallelList = new LinkedList<>();
    public static LinkedList<BetterTask> infiniteList = new LinkedList<>();
    public static boolean executeTaskCondition = false;

    public static void schedule(BetterTask task){
        if(task.getType().equals(TaskType.SEQUENTIAL)) list.addLast(task);
        else if(task.getType().equals(TaskType.PARALLEL)) parallelList.addLast(task);
        else if(task.getType().equals(TaskType.PARALLEL)) infiniteList.addLast(task);
    }

    public static void update(){
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
