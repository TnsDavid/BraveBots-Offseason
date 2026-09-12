package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso;

import java.util.LinkedList;
import java.util.function.Consumer;

public class CommandGroup {
    protected LinkedList<BetterTask> commandGroup;
    protected Consumer<Boolean> consumer;
    public CommandGroup(Consumer<Boolean> consumer){
        commandGroup = new LinkedList<>();
        this.consumer = consumer;
    }

    public CommandGroup addTask(Runnable runnable){
        BetterTask task = BetterTask.createSimpleTask(runnable, TaskType.SEQUENTIAL);
        commandGroup.addLast(task);
        return this;
    }

    public CommandGroup addTask(BetterTask task){
        commandGroup.addLast(task);
        return this;
    }

    public LinkedList<BetterTask> getCommand(){
        if(consumer != null){
            commandGroup.addLast(BetterTask.createTask()
                    .setTaskEnd(consumer)
                    .setTaskType(TaskType.SEQUENTIAL)
            );
        }
        return commandGroup;
    }
}
