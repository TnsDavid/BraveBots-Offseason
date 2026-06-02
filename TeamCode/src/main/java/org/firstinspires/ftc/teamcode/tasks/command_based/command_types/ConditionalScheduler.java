package org.firstinspires.ftc.teamcode.tasks.command_based.command_types;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

import java.util.function.BooleanSupplier;

public class ConditionalScheduler implements Task {



    public enum Cardinality{
        TWO, MORE_THAN_TWO
    }
    Cardinality cardinality;
    SchedulerWithCondition[] schedulers;
    Command commandIfTrue, commandIfFalse, finalCommand;
    BooleanSupplier supplier;

    boolean checked= false;
    public ConditionalScheduler(Command commandIfTrue, Command commandIfFalse, BooleanSupplier supplier){
        this.commandIfFalse = commandIfFalse;
        this.commandIfTrue = commandIfTrue;
        this.supplier= supplier;
        cardinality= Cardinality.TWO;
    }

    public ConditionalScheduler(Command defaultCommand, SchedulerWithCondition... schedulers){
        this.schedulers= schedulers;
        this.finalCommand = defaultCommand;
        cardinality= Cardinality.MORE_THAN_TWO;
    }





    @Override
    public boolean Run() {


        if(!checked){
            if(cardinality== Cardinality.TWO)
            {
                boolean res= supplier.getAsBoolean();
                if(res)
                    finalCommand = commandIfTrue;
                else
                    finalCommand = commandIfFalse;
                checked= true;
            }
            else{
                checked= true;
                for(SchedulerWithCondition scheduler: schedulers){
                    boolean res= scheduler.supplier.getAsBoolean();
                    if(res){
                        this.finalCommand = scheduler.command;
                        break;
                    }
                }
            }
        }

        finalCommand.update();
        return finalCommand.done();
    }


}
