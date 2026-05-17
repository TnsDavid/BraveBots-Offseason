package org.firstinspires.ftc.teamcode.tasks.type;

import org.firstinspires.ftc.teamcode.tasks.core.Task;

import java.util.function.BooleanSupplier;

public class TaskWithCondition {
    Task task;
    public BooleanSupplier supplier;

    public TaskWithCondition(Task task , BooleanSupplier supplier){
        this.task = task;
        this.supplier = supplier;
    }
}
