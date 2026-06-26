package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.Hang;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class HangTask implements Task {
    Command command;
    ElapsedTime timer;

    public HangTask(){
        command = new Command()
                .addTask(() -> Hang.raised = false)
                .addTask(() -> timer = new ElapsedTime())
                .addTask(() -> {
                    Hang.hang();
                    return timer.seconds() > 10;
                })
                .addTask(Hang::stopHang)
                .addTask(() -> Hang.raised = true);
    }

    @Override
    public boolean Run(){
        command.update();
        return command.done();
    }
}
