package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Storage {
    public static BetterMotorEx transfer;
    public static DigitalChannel up, middle , down ;
    public static boolean activestorage;
    public static ElapsedTime timer ;


    public enum Status{
        EMPTY , ONEBALL, TWOBALL , STORAGEFULL;

    }
    public static  Status status =Status.EMPTY;
    public static void start(){
        transfer.setPower(0.94);
        timer = new ElapsedTime();
        activestorage = true;
    }

    public static void stop(){
        transfer.setPower(0);
        activestorage = false;
    }

    public static void toggle(){
        if(activestorage) stop();
        else start();
    }

    public static void forceStop(){
        transfer.setPower(0);
        activestorage = false;
        Intake.stop();
        timer.reset();
    }

    public static Status getState() {
        if (timer.seconds() > 0.3) {
            if (!down.getState() && !middle.getState() && up.getState()) {
                status = Status.ONEBALL;
            } else if(!down.getState()&& middle.getState() && up.getState()){
                status = Status.TWOBALL;
            }
            else status = Status.STORAGEFULL;
        }
        return status;
    }

    public static void StorageForcedStop(){
        if(getState().equals(Status.STORAGEFULL)) stop();
    }

}