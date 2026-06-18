package org.firstinspires.ftc.teamcode.tasks.command_based.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToConstantAsync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToConstantAsyncReversed;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToConstantSync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToLinearAsync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToLinearSync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToTangentialAsync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToTangentialAsyncReversed;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToTangentialSync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.CurveToTangentialSyncReversed;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToConstantAsync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToConstantAsyncTValue;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToConstantSync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToLinearAsync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToLinearSync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToTangentialASyncTValue;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToTangentialAsync;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToTangentialAsyncReversed;
import org.firstinspires.ftc.teamcode.tasks.auto_task.LineToTangentialSync;
import org.firstinspires.ftc.teamcode.tasks.command_based.command_types.Wait;

import java.util.LinkedList;

public class Command {
    Follower f;
    LinkedList<TaskFactory> factory;

    boolean last= true, current= false, justDone= false;
    public LinkedList<Task> list;

    public Command(){
        list = new LinkedList<>();
        factory= new LinkedList<>();

    }
    public Command setChassis(Follower f){
        this.f= f;
        return this;
    }
    public Command(Follower f) {
        this.f =f;
        list = new LinkedList<>();
        factory= new LinkedList<>();
    }
    public boolean done(){
        return list.isEmpty();
    }
    @Deprecated
    public int getQueueSize(){
        return list.size();
    }

    public Command addChassis(Follower chassis){
        this.f= chassis;
        return this;
    }
    public Command addTask(Task t){
        factory.addLast(()-> t);
        list.addLast(t);
        return this;
    }
    public Command addTask(InstantTask t){

        list.addLast(()-> {
            t.run();
            return true;
        });
        factory.addLast(()-> ()->{
            t.run();
            return true;
        });
        return this;
    }

    @Deprecated
    public boolean justDone(){
        return justDone;
    }
    public Command waitSeconds(double sec){


        list.addLast(new Wait(sec));

        return this;
    }
    @Deprecated
    public Command waitSecondsFirst(double sec){
        list.addFirst(new Wait(sec));
        return this;
    }

    public void removeAllTasks(){
        list = new LinkedList<>();
    }
    @Deprecated
    public Command addFirst(Task t){
        list.addFirst(t);
        return this;
    }



    public void update(){

        if(done()) {
            current= true;
            justDone= !last;
            last= true;
            return;
        }
        current= false;
        last= false;


        Task t= list.getFirst();
        boolean result= t.Run();
        if(result)
            list.removeFirst();

    }
    public Command lineToConstantAsync(Pose p, double heading){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToConstantAsync(f, p, heading));
        return this;
    }
    public Command lineToConstantAsync(Pose p, double heading,double tValue){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToConstantAsyncTValue(f, p, heading,tValue));
        return this;
    }

    public Command lineToConstantSync(Pose p, double heading){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToConstantSync(f, p));
        return this;
    }
    public Command lineToLinearAsync(Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToLinearAsync(f, p));
        return this;
    }
    public Command lineToLinearSync(Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToLinearSync(f, p));
        return this;
    }

    public Command lineToTangentialASync(Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToTangentialAsync(f, p));
        return this;
    }
    public Command lineToTangentialSync(Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToTangentialSync(f, p));
        return this;
    }
    public Command curveToTangentialAsync(Pose controlPoint ,Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToTangentialAsync(f, controlPoint, p));
        return this;
    }
    public Command curveToTangentialSync(Pose controlPoint ,Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToTangentialSync(f, controlPoint, p));
        return this;
    }
    public Command curveToTangentialSyncReversed(Pose controlPoint ,Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToTangentialSyncReversed(f, controlPoint, p));
        return this;
    } public Command curveToTangentialAsyncReversed(Pose controlPoint ,Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToTangentialAsyncReversed(f, controlPoint, p));
        return this;
    }
    public Command curveToConstantAsync(Pose controlPoint ,Pose p, double heading){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToConstantAsync(f, controlPoint, p, heading));
        return this;
    }public Command curveToConstantAsyncReversed(Pose controlPoint ,Pose p, double heading){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToConstantAsyncReversed(f, controlPoint, p, heading));
        return this;
    }

    public Command lineToTangentialAsyncReversed(Pose p){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToTangentialAsyncReversed(f, p));
        return this;
    }
    public Command curveToLinearAsync(Pose controlPoint, Pose p)
    {
        if(f == null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToLinearAsync(f, controlPoint, p));
        return this;
    }
    public Command curveToLinearSync(Pose controlPoint, Pose p)
    {
        if(f == null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToLinearSync(f, controlPoint, p));
        return this;
    }
    public Command curveToConstantSync(Pose controlPoint, Pose p, double heading){
        if(f== null)
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new CurveToConstantSync(f, controlPoint, p, heading));
        return this;
    }
    public Command lineToTangentialASyncTValue(Pose p, double tValue) {
        if( f == null )
            throw new IllegalArgumentException("Follower not set");
        list.addLast(new LineToTangentialASyncTValue(f, p, tValue));
        return this;
    }




}
