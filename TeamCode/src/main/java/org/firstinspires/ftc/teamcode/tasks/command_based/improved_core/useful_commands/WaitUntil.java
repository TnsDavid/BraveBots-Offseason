package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

import java.util.function.BooleanSupplier;

public class WaitUntil {
    public static BetterTask command;
    public static void createCommand(BooleanSupplier condition){
        command = BetterTask.createTask()
                .setTaskJustDone(condition)
                .setTaskType(TaskType.SEQUENTIAL);
    }

    public static BetterTask getCommand(){
        return command;
    }
}
