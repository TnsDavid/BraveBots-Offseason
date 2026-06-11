package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

@Configurable
public class ShootDynamic implements Task {
    private Command s;

    public ShootDynamic(){
        s= new Command()
                .addTask(() -> Shooter.readyForShoot = true )
                .addTask(Blocker::unblock)
                .waitSeconds(0.3)
                .addTask(Intake::start)
                .addTask(Storage::start)
                .waitSeconds(1)
                .addTask(Storage::stop)
                .addTask(Intake::stop)
                .addTask(Blocker::block)
                .addTask(() -> {
                    Storage.count = 0;
                    Storage.startedTimer = false;
                    Shooter.readyForShoot = false;
                });
    }

    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
