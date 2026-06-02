package org.firstinspires.ftc.teamcode.tasks.command_based.core;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.Supplier;

public class Scheduler {
    public static void clearParallels(){
        parallels.clear();
    }
    private static final LinkedList<Task> list= new LinkedList<>();
    public static void schedule(Supplier<? extends Task> factory){
        list.addLast(factory.get());
    }
    public static void clear(){
        list.clear();
    }

    public static void schedule(Task t){
        list.addLast(t);
    }
    public static void now(Task t){
        clear();
        list.addLast(t);
    }
    public static void now(InstantTask t){
        clear();
        list.addLast(()->{
            t.run();
            return true;
        });
    }
    public static void now(Supplier<? extends Task> supplier){
        clear();
        list.addLast(supplier.get());
    }
    public static void schedule(InstantTask t){
        Task task= ()->{
            t.run();
            return true;
        };
        list.addLast(task);

    }
    public static void schedule(Task... t){
        for(Task task: t){
            list.addLast(task);
        }
    }
    public static void schedule(InstantTask... t){
        for(InstantTask task: t){
            Task tasknormal= ()->{
                task.run();
                return true;
            };
            list.addLast(tasknormal);
        }
    }
    public static void inParallel(Task t){
        LinkedList<Task> parallel = new LinkedList<>();
        parallel.addLast(t);
        parallels.addLast(parallel);
    }
    public static void inParallel(Supplier<? extends Task> t){
        LinkedList<Task> parallel = new LinkedList<>();
        parallel.addLast(t.get());
        parallels.addLast(parallel);
    }
    public static LinkedList<LinkedList<Task>> parallels= new LinkedList<>();
    public static void update(){
        if(!list.isEmpty()) {
            if (list.getFirst().Run()) list.removeFirst();
        }
        if(parallels.isEmpty()) return;
        Iterator<LinkedList<Task>> it= parallels.iterator();
        while (it.hasNext())
        {
            LinkedList<Task> q= it.next();
            if(q.isEmpty())
            {
                it.remove();
                continue;
            }
            if(q.getFirst().Run()) q.removeFirst();
            if(q.isEmpty()) it.remove();
        }
    }

}
