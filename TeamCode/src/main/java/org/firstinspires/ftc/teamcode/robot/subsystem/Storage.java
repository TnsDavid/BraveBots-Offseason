package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;

@Configurable
public class Storage {
    public static BetterMotorEx transfer;
    public static BreakBeam bUp, bDown;
    public static double last, updateTime = 100;
    public enum Status {
        EMPTY, ONEBALL, TWOBALL, STORAGEFULL

    }
    public enum ActiveStatus{
        TRANSFER, NOTRANSFER
    }
    public static Status status = Status.EMPTY;
    public static ActiveStatus activeStatus = ActiveStatus.NOTRANSFER;
    public static void start() {
        transfer.setPower(1);
        activeStatus = ActiveStatus.TRANSFER;
    }
    public static void stop() {
        transfer.setPower(0);
        activeStatus = ActiveStatus.NOTRANSFER;
    }
    public static int count = 0;
    public static ElapsedTime ballUp;
    public static boolean startedTimer = false;
    public static double storageStopTime = 80;

    public static void update(){
        if(!Shooter.readyForShoot){
            if(!startedTimer && bUp.isBeamBroken()){
                ballUp = new ElapsedTime();
                startedTimer = true;
            }
            if(ballUp != null && ballUp.milliseconds()>storageStopTime && startedTimer) stop();
        }
        if(bDown.isBeamBroken() && System.currentTimeMillis() - last > updateTime){
            if(count < 3) count++;
            last = System.currentTimeMillis();
        }
        switch(count){
            case 0:
                status = Status.EMPTY;
                break;
            case 1:
                status = Status.ONEBALL;
                break;
            case 2:
                status = Status.TWOBALL;
                break;
            case 3:
                status = Status.STORAGEFULL;
                break;
        }
//        if(activeStatus.equals(ActiveStatus.TRANSFER)) start();
//        else stop();
    }
}