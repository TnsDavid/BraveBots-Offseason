package org.firstinspires.ftc.teamcode.tasks.command_based.core;

import java.util.LinkedList;
import java.util.function.Consumer;

public class ConsumerCommand {
    public LinkedList<InstantTask> list;
    public Consumer<InstantTask> mainConsumer;
    public ConsumerCommand(){
        mainConsumer = InstantTask::run;
    }

    public ConsumerCommand addTask(InstantTask t){
        list.addLast(t);
        return this;
    }

    public ConsumerCommand addTasks(InstantTask... t){
        for(InstantTask task : t){
            list.addLast(task);
        }
        return this;
    }

    public void runTasks() {
        for (InstantTask task : list) {
            mainConsumer.accept(task);
        }
    }
}
