package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class ShootFar implements Task {
    Command command ;

    public static double treshhold = 80;
    public static double totalShootTime=0.5;
    public ShootFar() {
        command = new Command()
                .addTask(() -> {
                    if (Storage.status.equals(Storage.Status.ONEBALL)) Storage.oneBall = true;
                })
                .addTask(() -> Math.abs(Shooter.motor2.getVelocity()) > Shooter.velocityFAR - treshhold)
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .waitSeconds(0.1)
                .addTask(Intake::start)
                .addTask(Storage::start)
                .waitSeconds(totalShootTime)
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
    @Override
    public boolean Run() {
        command.update();
        return command.done();
    }
}
