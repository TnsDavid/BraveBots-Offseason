package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.useful_commands;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.TaskType;

import java.util.function.BooleanSupplier;

public class Conditional {
    public static BetterTask getCommand(Runnable ifTrue, Runnable ifFalse, BooleanSupplier condition, TaskType type){
        BetterTask command;
        if(condition.getAsBoolean()) command = BetterTask.createSimpleTask(ifTrue, type);
        else command = BetterTask.createSimpleTask(ifFalse, type);

        return command;
    }

    public static BetterTask getCommand(BetterTask ifTrue, BetterTask ifFalse, BooleanSupplier condition){
        BetterTask command;
        if(condition.getAsBoolean()) command = ifTrue;
        else command = ifFalse;

        return command;
    }
}
