package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.opmodes.Test;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.util.Ghiocel;


@Ghiocel
public class ptGaoc implements Task {
     Command command;
     public ptGaoc() {
         command = new Command();
         command.addTask(Shooter::setReady)
                 .waitSeconds(.2)
                 .addTask(Blocker::unblock)
                 .waitSeconds(1)
                 .addTask(Blocker::block)
                 .addTask(Shooter::setReady);
     }
     @Override
     public boolean Run() {
         command.update();
         return command.done();
     }
}
