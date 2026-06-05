package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.Test;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Storage {
    public static BetterMotorEx transfer;
    public static DigitalChannel up, down ;
    public static boolean activestorage;
    public static ElapsedTime timer;
    public static double last, minimumUpdateTime = 300;


    public enum Status{
        EMPTY(0) , ONEBALL(1), TWOBALL(2) , STORAGEFULL(3);
        final double nustiu;
        Status(double nustiu){
            this.nustiu = nustiu;
        }

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
        if (System.currentTimeMillis() - last > minimumUpdateTime) {
            if (!down.getState() && up.getState())
                status = Status.ONEBALL;
            else if( down.getState() && up.getState() )
                status = Status.STORAGEFULL;
            else
                status = Status.EMPTY;
            last = System.currentTimeMillis();
        }
        return status;
    }

    public static void StorageForcedStop(){
        if(getState().equals(Status.STORAGEFULL)) stop();
    }

    public static void checkUpLaser(){
        if(up.getState()) stop();
    }
    public static void update() {
        getState();
        if( !status.equals(Status.EMPTY) && !Shooter.readyForShoot )
            stop();
        else if( !status.equals(Status.EMPTY) && Shooter.readyForShoot )
            start();
    }

}