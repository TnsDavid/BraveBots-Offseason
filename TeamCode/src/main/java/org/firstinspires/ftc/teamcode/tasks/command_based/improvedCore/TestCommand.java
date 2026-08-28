package org.firstinspires.ftc.teamcode.tasks.command_based.improvedCore;

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
