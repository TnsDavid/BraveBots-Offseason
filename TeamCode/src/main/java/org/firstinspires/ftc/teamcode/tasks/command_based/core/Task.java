package org.firstinspires.ftc.teamcode.tasks.command_based.core;

public interface Task {
    default void end(boolean interrupted){};
    boolean Run();
}
