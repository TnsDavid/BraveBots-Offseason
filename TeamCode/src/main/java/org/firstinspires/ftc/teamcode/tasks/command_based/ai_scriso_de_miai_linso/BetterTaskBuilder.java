package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso;

import static org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.ExecuteType.ONCE;
import static org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.TaskType.SEQUENTIAL;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class BetterTaskBuilder implements BetterTask {
    public Consumer<Boolean> end;
    public Runnable execute;
    public Runnable start;
    public BooleanSupplier justDone;
    public TaskType type;
    public ExecuteType executeType;

    private BetterTaskBuilder(
            Consumer<Boolean> end,
            Runnable execute,
            Runnable start,
            BooleanSupplier justDone,
            TaskType type,
            ExecuteType executeType
    ) {
        this.end = end;
        this.execute = execute;
        this.start = start;
        this.justDone = justDone;
        this.type = type;
        this.executeType = executeType;
    }

    public BetterTaskBuilder() {
        this(
                emptyBooleanConsumer,
                emptyRunnable,
                emptyRunnable,
                emptyBooleanSupplier,
                SEQUENTIAL,
                ONCE
        );
    }

    public static final BooleanSupplier emptyBooleanSupplier = () -> true;
    public static final Runnable emptyRunnable = () -> {};
    public static final Consumer<Boolean> emptyBooleanConsumer = condition -> {};

    public BetterTaskBuilder setTaskExecute(Runnable execute){
        this.execute = execute;
        return this;
    }

    public BetterTaskBuilder setTaskStart(Runnable start){
        this.start = start;
        return this;
    }

    public BetterTaskBuilder setTaskEnd(Consumer<Boolean> end){
        this.end = end;
        return this;
    }

    public BetterTaskBuilder setTaskJustDone(BooleanSupplier justDone){
        this.justDone = justDone;
        return this;
    }

    public BetterTaskBuilder setExecuteType(ExecuteType executeType){
        this.executeType = executeType;
        return this;
    }

    public BetterTaskBuilder setTaskType(TaskType type){
        this.type = type;
        return this;
    }

    @Override
    public void end(boolean interrupted){
        end.accept(interrupted);
    }
    @Override
    public void execute(){
        execute.run();
    }
    @Override
    public void start(){
        start.run();
    }
    @Override
    public boolean done(){
        return justDone.getAsBoolean();
    }
    @Override
    public TaskType getType(){return this.type;}
    @Override
    public ExecuteType getExecuteType(){return this.executeType;}
}
