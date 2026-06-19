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
    public static double coefFar = 1.4, waitTime= 0.01, waitTime2= 0.02, incrementFar = 0.08;

    public ShootFar(){
        command = new Command()
                .addTask(()-> Math.abs(Shooter.motor1.getVelocity()) > Shooter.velocityFAR - treshhold )
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .waitSeconds(0.1)
                .addTask(Intake::start)
                .addTask(Storage::start)
                .waitSeconds(waitTime)
                .addTask(()->{
                    Shooter.servo.setPosition(Shooter.hoodFAR- incrementFar);
                })
                .waitSeconds(waitTime2)
                .addTask(()->{
                    Shooter.servo.setPosition(Shooter.hoodFAR-coefFar* incrementFar);
                })
                .waitSeconds(1)
                .addTask(Blocker::block)
                .addTask(() -> {
                    Turret.turrStatus= Turret.TurretStatus.NOTACTIVE;
                    Storage.count = 0;
                    Storage.startedTimer = false;
                    Shooter.readyForShoot = false;
                    Turret.checkedStatus = false;
                })
                .addTask(()-> Shooter.servo.setPosition(Shooter.hoodFAR));    }
    @Override
    public boolean Run() {
        command.update();
        return command.done();
    }
}
