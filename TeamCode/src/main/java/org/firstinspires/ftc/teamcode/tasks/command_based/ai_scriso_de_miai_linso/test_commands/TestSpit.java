package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.test_commands;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterScheduler;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.CommandGroup;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.useful_commands.WaitSeconds;

import java.util.LinkedList;
import java.util.function.Consumer;

public class TestSpit {
    public static CommandGroup commandGroup;
    public static void createCommandGroup(){
        Consumer<Boolean> end = interrupted -> {
            if(interrupted) Intake.stop();
        };
        commandGroup = new CommandGroup(end)
                .addTask(Intake::reverse)
                .addTask(WaitSeconds.getCommand(.2))
                .addTask(Intake::stop);
    }

    public static LinkedList<BetterTask> getCommand(){
        return commandGroup.getCommand();
    }
}
