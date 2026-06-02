package org.firstinspires.ftc.teamcode.tasks.command_based.command_types;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

import java.util.function.BooleanSupplier;

public class TaskWithCondition {

    public Task task;
    public BooleanSupplier supplier;
    public TaskWithCondition(Task task, BooleanSupplier supplier){
        this.task= task;
        this.supplier= supplier;
    }
}
