package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.test_commands;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.ExecuteType;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.TaskType;

public class TestCommand {
    public static BetterTask testTask;
    public static void createCommand(){
        testTask = BetterTask.createTask()
                .setTaskStart(() -> {System.out.println("start");})
                .setTaskEnd(interrupted -> {
                    if(!interrupted) UnCacat.count = 0;
                })
                .setTaskExecute(() -> {
                    UnCacat.count++;
                    System.out.println(UnCacat.count);
                })
                .setExecuteType(ExecuteType.CONTINUOUS)
                .setTaskType(TaskType.SEQUENTIAL)
                .setTaskJustDone(() -> UnCacat.count >= 3);
    }

    public static BetterTask getCommand(){
        return testTask;
    }
}
