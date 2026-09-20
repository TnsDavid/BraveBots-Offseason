package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.useful_commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.TaskType;

public class WaitSeconds {
    public static BetterTask getCommand(double seconds){
        ElapsedTime timer = new ElapsedTime();
        return BetterTask.createTask()
                .setTaskJustDone(() -> timer.seconds() > seconds)
                .setTaskType(TaskType.SEQUENTIAL);
    }
}
