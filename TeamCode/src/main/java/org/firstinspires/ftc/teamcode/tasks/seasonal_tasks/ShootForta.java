package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

public class ShootForta implements Task {
    private Command s;

    public static double  velocityThreshold = 300,coef = 2, increment= -0.013 , waitTime= 0.23, waitTime2= 0.12;


    public double pos;

    public ShootForta(){
        s= new Command()
                .addTask(()-> {
                    Intake.relanti();
                    pos = Shooter.servo.getPosition();
                })
                .addTask(Storage::start)
                .waitSeconds(waitTime)
                .addTask(()->Shooter.servo.setPosition(pos+increment))
                .waitSeconds(waitTime2)
                .addTask(()->Shooter.servo.setPosition(pos+coef* increment))
                .waitSeconds(1)
                .addTask(Storage::stop)
                .addTask(()->{
                    Shooter.servo.setPosition(pos);
                });
    }

    @Override
    public boolean Run() {
        s.update();
        return s.done();
    }
}
