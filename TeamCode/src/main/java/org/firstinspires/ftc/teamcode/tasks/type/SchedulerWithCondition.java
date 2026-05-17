package org.firstinspires.ftc.teamcode.tasks.type;

import org.firstinspires.ftc.teamcode.tasks.core.Command;

import java.util.function.BooleanSupplier;

public class SchedulerWithCondition {
    public Command command;
    public BooleanSupplier supplier;
    public SchedulerWithCondition(Command command, BooleanSupplier supplier){
        this.command = command;
        this.supplier= supplier;
    }
}
