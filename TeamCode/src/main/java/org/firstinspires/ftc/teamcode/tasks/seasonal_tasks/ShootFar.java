package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double coefFar = 1.4, waitTime= 0.01, totalShootTime=0.05, waitTime2= 0.02, incrementFar = 0.04;

    ElapsedTime timer;

    public ShootFar(){
        command = new Command()
                .addTask(()-> Math.abs(Shooter.motor1.getVelocity()) > Shooter.velocityFAR - treshhold )
                .addTask(() -> Shooter.readyForShoot = true)
                .addTask(Blocker::unblock)
                .waitSeconds(0.1)
                .addTask(Intake::start)
                .addTask(Storage::start)
                .addTask(()-> timer=new ElapsedTime())
                .addTask(() -> Storage.status.equals(Storage.Status.TWOBALL)||(timer.seconds()>totalShootTime))
                .addTask(()->{
                    Shooter.hoodServo.setPosition(Shooter.hoodFAR- incrementFar);
                })
                .addTask(()-> timer=new ElapsedTime())
                .addTask(() -> Storage.status.equals(Storage.Status.ONEBALL)||(timer.seconds()>totalShootTime))
                .addTask(()->{
                    Shooter.hoodServo.setPosition(Shooter.hoodFAR-coefFar* incrementFar);
                })
                .addTask(() -> Storage.status.equals(Storage.Status.EMPTY))
                .waitSeconds(0.1)
                .addTask(Blocker::block)
                .addTask(() -> {
                    Turret.turrStatus= Turret.TurretStatus.NOTACTIVE;
                    Storage.status= Storage.Status.EMPTY;
                    Storage.startedTimer = false;
                    Shooter.readyForShoot = false;
                    Turret.checkedStatus = false;
                })
                .addTask(()-> Shooter.hoodServo.setPosition(Shooter.hoodFAR));    }
    @Override
    public boolean Run() {
        command.update();
        return command.done();
    }
}
