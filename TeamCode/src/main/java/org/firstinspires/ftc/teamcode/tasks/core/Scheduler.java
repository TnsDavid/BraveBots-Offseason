package org.firstinspires.ftc.teamcode.tasks.core;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.Supplier;

public class Scheduler {

    public static final LinkedList<Task> list = new LinkedList<>();
    public static void schedule(Supplier<? extends Task> factory){list.addLast(factory.get());}

    public static void clear(){list.clear();}
    public static void now(Task t){
         clear();
         list.addLast(t);
    }
    public static void now(Supplier<? extends Task> supplier){
        clear();
        list.addLast(supplier.get());
    }
    public static void now(InstantTask t){
        clear();
        list.addLast(()->{
            t.run();
            return true;
        });
    }

    public static void inParalel(Task t) {
        LinkedList<Task> parallel = new LinkedList<>();
        parallel.addLast(t);
        parallels.addLast(parallel);
    }
    public static void inParalel(Supplier<? extends Task> t) {
        LinkedList<Task> parallel = new LinkedList<>();
        parallel.addLast(t.get());
        parallels.addLast(parallel);
    }

    public static LinkedList<LinkedList<Task>> parallels= new LinkedList<>();
    public static void update(){
        if(!list.isEmpty())
            if(list.getFirst().Run()) list.removeFirst();
        if(parallels.isEmpty()) return;
        Iterator<LinkedList<Task>> it = parallels.iterator();
        while(it.hasNext()){
            LinkedList<Task> q = it.next();
            if(q.isEmpty()){
                it.remove();
                continue;
            }
            if(q.getFirst().Run()) q.removeFirst();
            if(q.isEmpty()) it.remove();
        }
    }

}
