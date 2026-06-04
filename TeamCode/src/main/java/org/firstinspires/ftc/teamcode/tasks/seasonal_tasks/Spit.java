package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class Spit implements Task {
    Command command;
    public Spit(){
        command= new Command()
                .addTask(Intake::reverse)
                .waitSeconds(.4)
                .addTask(Intake::stop);
    }
    @Override
    public boolean Run() {
        command.update();
        return command.done();
    }
}