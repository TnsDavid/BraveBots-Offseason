package org.firstinspires.ftc.teamcode.tasks.seasonal_tasks;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.command_types.ConditionalTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;


// TODO: ASTA E PT VIITOR CAND AI 3 BREAKBEAM-URI
public class SmartIntake implements Task {

    private Command command;

    public SmartIntake(){
        command = new Command()
                .addTask(new ConditionalTask(()->{
                    Intake.start();
                    Storage.start();
                    return true;
                } , ()->{
                    Intake.start();
                    return true;
                        } , ()->Storage.status.equals(Storage.Status.TWOBALL))
                );
    }
    @Override
    public boolean Run() {
        return false;
    }
}
