package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

@Configurable
public class Shoot implements Task {
    private Command s;

    public static double coef = 1.4, coefFar = 1.25, increment= 0.009, waitTime= 0.01, waitTime2= 0.05, incrementFar = 0.024, inc;


    public double pos;

    public Shoot(){
        s= new Command()
                .addTask(() -> pos = Shooter.servo.getPosition())
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .waitSeconds(0.3)
                .addTask(Intake::start)
                .addTask(Storage::start)
                .waitSeconds(waitTime)
                .addTask(()->{
                    inc = Robot.playStyle == Robot.PlayStyle.CLOSE ? increment : incrementFar;
                    if(!Turret.sotm) Shooter.servo.setPosition(pos- inc);
                })
                .waitSeconds(waitTime2)
                .addTask(()->{
                    double cf = Robot.playStyle == Robot.PlayStyle.CLOSE ? coef : coefFar;
                    if(!Turret.sotm) Shooter.servo.setPosition(pos-cf* inc);
                })
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