package org.firstinspires.ftc.teamcode.tasks.command_based.command_types;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

import java.util.function.Supplier;

public class ThreadedTask implements Task {
    Command command;
    boolean tRunning;
    Thread t;
    ElapsedTime timer;
    double secondsToWait;
    public ThreadedTask(Supplier<Task> s, double secondsToWait) {
        this.secondsToWait = secondsToWait;
        command = new Command()
                .addTask(s.get())
                .addTask(Intake::start)
                .addTask(Storage::start)
                .addTask(() -> {
                    tRunning = false;
                });
        t = new Thread(() -> {
            while( tRunning ) {
                try {
                    command.update();
                    sleep(3);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
    }

    @Override
    public boolean Run(){
        if(!tRunning){
            timer = new ElapsedTime();
            tRunning = true;
            t.start();
        }
        return timer.seconds() > this.secondsToWait;
    }
}
