package org.firstinspires.ftc.teamcode.tasks.command_based.core;

import org.threeten.bp.Instant;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class ConsumerCommand {
    public LinkedList<InstantTask> taskList;
    public LinkedList<BooleanSupplier> booleanList;
    public boolean done;
    public Consumer<InstantTask> mainConsumer;
    public ConsumerCommand(){
        mainConsumer = Runnable -> {

        };
        taskList = new LinkedList<>();
        booleanList = new LinkedList<>();
        done = false;
    }

    public ConsumerCommand addTask(InstantTask t){
        taskList.addLast(t);
        booleanList.addLast(() -> true);
        return this;
    }

    public ConsumerCommand addTasks(InstantTask... t){
        for(InstantTask task : t){
            taskList.addLast(task);
            booleanList.addLast(() -> true);
        }
        return this;
    }

    public ConsumerCommand addTask(InstantTask t, BooleanSupplier s){
        taskList.addLast(t);
        booleanList.addLast(s);
        return this;
    }

    public void update() {
        BooleanSupplier currentCondition = booleanList.getFirst();
        InstantTask currentTask = taskList.getFirst();
        if(currentCondition.getAsBoolean()){
            mainConsumer.accept(currentTask);
            taskList.removeFirst();
            booleanList.removeFirst();
        } else mainConsumer.accept(currentTask);
    }
}
