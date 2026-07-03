package org.firstinspires.ftc.teamcode.tasks.command_based.command_types;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.util.ElapsedTime;

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
        tRunning = true;
        command = new Command()
                .addTask(s.get())
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
        t.start();
        timer = new ElapsedTime();
    }

    @Override
    public boolean Run(){
        return timer.seconds() > this.secondsToWait;
    }
}
