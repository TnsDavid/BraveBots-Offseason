package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.useful_commands;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.TaskType;

import java.util.function.BooleanSupplier;

public class WaitUntil {
    public static BetterTask command;
    public static BetterTask getCommand(BooleanSupplier condition){
        return BetterTask.createTask()
                .setTaskJustDone(condition)
                .setTaskType(TaskType.SEQUENTIAL);
    }
}
