package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class ShootFar implements Task {
    Command command ;

    public static double treshhold = 100;
    public static double coef = 1.35, coefFar = 1.4, increment= 0.0082, waitTime= 0.01, waitTime2= 0.02, incrementFar = 0.08, inc;
    public double pos;

    public ShootFar(){
        command = new Command()
                .addTask(()-> Math.abs(Shooter.motor1.getVelocity()) > Shooter.velocityFAR - treshhold )
                .addTask(() -> pos = Shooter.servo.getPosition())
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .waitSeconds(0.1)
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
                    Turret.turrStatus= Turret.TurretStatus.NOTACTIVE;
                    Storage.count = 0;
                    Storage.startedTimer = false;
                    Shooter.readyForShoot = false;
                    Turret.checkedStatus = false;
                })
                .addTask(()-> Shooter.servo.setPosition(pos));    }
    @Override
    public boolean Run() {
        command.update();
        return command.done();
    }
}
