package org.firstinspires.ftc.teamcode.tasks.type;

import org.firstinspires.ftc.teamcode.tasks.core.Task;

import java.util.function.BooleanSupplier;

public class ConditionalTask implements Task {

    public enum Cardinality{
        TWO, MORE_THAN_TWO;
    }

    Cardinality cardinality = Cardinality.TWO;

    Task task, taskIfTrue, taskIfFalse;
    BooleanSupplier supplier;
    boolean checked = false;


    public ConditionalTask(Task ifTrue, Task ifFalse, BooleanSupplier supplier){
        this.taskIfTrue = ifTrue;
        this.taskIfFalse = ifFalse;
        this.supplier = supplier;
        cardinality = Cardinality.TWO;
    }

    TaskWithCondition[] conditionedTasks;
    public ConditionalTask(Task defaultTask,TaskWithCondition... tasks){
        this.task = defaultTask;
        conditionedTasks = tasks;
        cardinality = Cardinality.MORE_THAN_TWO;
    }

    @Override
    public boolean Run() {
        if(!checked){
            if(cardinality == Cardinality.TWO){
                boolean res =this.supplier.getAsBoolean();
                if(res)
                { this.task = taskIfTrue;}
                else this.task = taskIfFalse;

                checked = true;
            }
            else{
                checked = true;
                for(TaskWithCondition  taskWithCondition : conditionedTasks){
                    boolean res = this.supplier.getAsBoolean();
                    if(res){
                        this.task = taskWithCondition.task;
                        break;
                    }
                }
            }
        }

        return task.Run();
    }
}
