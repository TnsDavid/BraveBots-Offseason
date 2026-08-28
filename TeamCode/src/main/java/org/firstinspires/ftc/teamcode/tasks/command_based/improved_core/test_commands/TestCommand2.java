package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.test_commands;

import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.ExecuteType;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.TaskType;

public class TestCommand2 {
    public static BetterTask testTask2;
    public static void createCommand() {
        testTask2 = BetterTask.createTask()
                .setTaskEnd(interrupted -> {
                    if(!interrupted) System.out.println("end");
                })
                .setTaskExecute(() -> System.out.println("execute"))
                .setTaskJustDone(() -> false)
                .setTaskType(TaskType.PARALLEL)
                .setExecuteType(ExecuteType.CONTINUOUS);
    }

    public static BetterTask getCommand() {
        return testTask2;
    }
}
