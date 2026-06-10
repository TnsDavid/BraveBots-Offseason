package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

@Configurable
public class ShootTest implements Task {
    private Command s;

    public static double coef = 1.4, increment= 0.009, waitTime= 0.01, waitTime2= 0.05;


    public double pos;

    public ShootTest(){
        s= new Command()
                .addTask(() -> pos = Shooter.servo.getPosition())
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .waitSeconds(0.3)
                .addTask(Intake::start)
                .addTask(Storage::start)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.servo.setPosition(pos-increment))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.servo.setPosition(pos-coef* increment))
                .waitSeconds(1)
                .addTask(Storage::stop)
                .addTask(Intake::stop)
                .addTask(Blocker::block)
                .addTask(() -> {
                    Storage.count = 0;
                    Storage.startedTimer = false;
                    Shooter.readyForShoot = false;
                })
                .addTask(()-> Shooter.servo.setPosition(pos));
    }

    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
