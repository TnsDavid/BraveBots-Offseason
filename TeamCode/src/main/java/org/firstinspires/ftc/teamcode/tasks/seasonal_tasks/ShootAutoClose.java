package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class ShootAutoClose implements Task {
     public Command command;
     public double pos, coef = 1.35, increment = 0.082, waitTime = 0.01, waitTime2 = 0.02;
     public ShootAutoClose() {
         command = new Command();
         command.addTask(() -> pos = Shooter.servo.getPosition())
                 .addTask(() -> Shooter.readyForShoot = true)
                 .addTask(Blocker::unblock)
                 .waitSeconds(0.1)
                 .addTask(Intake::start)
                 .addTask(Storage::start)
                 .waitSeconds(waitTime)
                 .addTask(() -> Shooter.servo.setPosition(pos - increment))
                 .waitSeconds(waitTime2)
                 .addTask(() -> Shooter.servo.setPosition(pos - coef * increment))
                 .waitSeconds(0.05)
                 .addTask(Blocker::block)
                 .addTask(Storage::stop)
                 .addTask(Intake::stop)
                 .addTask(() -> {
                     Storage.status= Storage.Status.EMPTY;
                     Storage.startedTimer = false;
                     Shooter.readyForShoot = false;
                     Turret.checkedStatus = false;
                 })
                 .addTask(() -> Shooter.servo.setPosition(pos));

     }
     public boolean Run() {
         command.update();
         return command.done();
     }
}
