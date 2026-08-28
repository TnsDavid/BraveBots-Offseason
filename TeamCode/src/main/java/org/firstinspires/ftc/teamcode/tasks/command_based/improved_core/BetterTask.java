package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

public interface BetterTask {
    static BetterTaskBuilder createTask(){
        return new BetterTaskBuilder();
    }
    boolean done();
    void start();
    void end(boolean interrupted);
    void execute();
    ExecuteType getExecuteType();
    TaskType getType();
}