package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static double coef = 1.35, coefFar = 1.4,totalShootTime=0.05, increment= 0.0082, waitTime= 0.01, waitTime2= 0.02, incrementFar = 0.04, inc;


    public double pos;
    ElapsedTime timer;

    public Shoot(){
        if(Storage.isEmpty())
            s = new Command().addTask(Blocker::block);
        else
            s= new Command()
                    .addTask(() -> pos = Shooter.hoodServo.getPosition())
                    .addTask(() ->Shooter.readyForShoot = true)
                    .addTask(Blocker::unblock)
                    .waitSeconds(0.1)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(()-> timer=new ElapsedTime())
                    .addTask(() -> Storage.status.equals(Storage.Status.TWOBALL)||(timer.seconds()>totalShootTime))
                    .addTask(()-> timer=new ElapsedTime())
                    .addTask(() -> Storage.status.equals(Storage.Status.ONEBALL)||(timer.seconds()>totalShootTime))
                    .addTask(Storage::isEmpty)
                    .waitSeconds(0.05)
                    .addTask(()->{
                        Storage.stop();
                        Intake.stop();
                        Blocker.block();
                        return true;
                    })
                    .addTask(() -> {
                        Turret.turrStatus= Turret.TurretStatus.NOTACTIVE;
                        Storage.status= Storage.Status.EMPTY;
                        Storage.startedTimer = false;
                        Shooter.readyForShoot = false;
                        Turret.checkedStatus = false;
                        return true;
                    })
                    .addTask(()-> Shooter.hoodServo.setPosition(pos));
    }

    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}