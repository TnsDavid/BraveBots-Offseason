package org.firstinspires.ftc.teamcode.tasks.command_based.core;

import java.util.function.BooleanSupplier;

public interface Task {
    default void end(boolean interrupted){};
    boolean Run();
}
