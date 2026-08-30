package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class GamePadUpdate {
    public static BetterTask command;

    public static void createCommand(BestGamepad gp1, BestGamepad gp2){
        command = BetterTask.createTask()
                .setTaskExecute(() -> {
                    gp1.update();
                    gp2.update();
                })
                .setTaskType(TaskType.INFINITE);
    }

    public static BetterTask getCommand(){
        return command;
    }
}
