package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.ConsumerCommand;

public class shoot {
    ConsumerCommand command;
    public shoot(){
        command = new ConsumerCommand()
                .addTask(() -> System.out.println(1))
                .addTask(() -> System.out.println(2))
                .addTask(() -> System.out.println(3));
    }

    public void runCommand(){
        command.runTasks();
    }
}
