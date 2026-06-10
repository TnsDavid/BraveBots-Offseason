package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class ShootFarTest implements Task {

    private Command command;

    public ShootFarTest() {
        command = new Command();
        command.addTask(() -> Shooter.readyForShoot = true)
                .addTask(Intake::start)
                .addTask(Blocker::unblock)
                .addTask(Storage::start)
                .waitSeconds(0.013)
                .addTask(Storage::stop)
                .waitSeconds(0.3)
                .addTask(Storage::start)
                .waitSeconds(0.013)
                .addTask(Storage::stop)
                .waitSeconds(0.3)
                .addTask(Storage::start)
                .waitSeconds(0.02)
                .addTask(Storage::stop)
                .addTask(Intake::stop)
                .addTask(Blocker::block)
                .addTask(() -> Shooter.readyForShoot = false);
    }
    @Override
    public boolean Run() {
        command.update();
        return command.done();
    }
}
