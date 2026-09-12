package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.test_commands;

import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterScheduler;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.CommandGroup;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.useful_commands.WaitSeconds;

import java.util.LinkedList;
import java.util.function.Consumer;

public class TestShoot{
    public static CommandGroup commandGroup;
    public static double totalShootTime=0.7;
    public static void createCommandGroup(){
        Consumer<Boolean> end = interrupted -> {
            Blocker.block();
            if(interrupted){
                Intake.stop();
                Storage.stop();
            }
        };
        commandGroup = new CommandGroup(end)
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .addTask(WaitSeconds.getCommand(0.1))
                .addTask(Intake::start)
                .addTask(Storage::start)
                .addTask(WaitSeconds.getCommand(totalShootTime))
                .addTask(Storage::stop)
                .addTask(Intake::stop)
                .addTask(Blocker::block)
                .addTask(() -> {
                    Storage.startedTimer = false;
                    Shooter.readyForShoot = false;
                    Turret.checkedStatus = false;
                    Storage.oneBall = false;
                });
    }

    public static LinkedList<BetterTask> getCommand(){
        return commandGroup.getCommand();
    }
}
