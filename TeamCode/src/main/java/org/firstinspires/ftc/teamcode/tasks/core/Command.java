package org.firstinspires.ftc.teamcode.tasks.core;


import org.firstinspires.ftc.teamcode.tasks.type.Wait;

import java.util.LinkedList;

public class Command {

    LinkedList<TaskFactory> factory = new LinkedList<>();
    LinkedList<Task> list = new LinkedList<>();

    boolean last = true, current = false , justDone = false;

    public Command(){
        list = new LinkedList<>();
        factory = new LinkedList<>();
    }

    public boolean done(){
        return list.isEmpty();
    }

    public Command addTask(Task t){
        list.addLast(t);
        factory.addLast(()->t);
        return this;
    }

    public Command waitSeconds(double sec){
        list.addLast(new Wait(sec));
        return this;
    }

    public Command addTask(InstantTask t){
        list.addLast(()->
        {
            t.run();
            return true;
        });
        factory.addLast(()->()->{
            t.run();
            return true;
        });
        return this;
    }
    public Command RemoveAllTasks(){list = new LinkedList<>(); return this;}

    public Command addFirst(Task t){
        list.addFirst(t);
        return this;
    }
    public void update(){
        if(done()){
        current = true;
        justDone = !last;
        last = true;
        }
        current = false;
        last = false;

        Task t = list.getFirst();
        boolean res = t.Run();
        if(res){
            list.removeFirst();
        }
    }
}
