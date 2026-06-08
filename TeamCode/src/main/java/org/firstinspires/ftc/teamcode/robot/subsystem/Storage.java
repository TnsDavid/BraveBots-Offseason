package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.Test;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;

@Configurable
public class Storage {
    public static BetterMotorEx transfer;
    public static DigitalChannel up;
    public static BreakBeam bup, bmiddle, bdown;
    public static boolean activestorage;
    public static ElapsedTime timer;
    public static double last, updateTime = 50;


    public enum Status {
        EMPTY, ONEBALL, TWOBALL, STORAGEFULL

    }

    public static Status status = Status.EMPTY;

    public static void start() {
        transfer.setPower(1);
        timer = new ElapsedTime();
        activestorage = true;
    }

    public static void stop() {
        transfer.setPower(0);
        activestorage = false;
    }

    public static void toggle() {
        if (activestorage) stop();
        else start();
    }

    public static void forceStop() {
        transfer.setPower(0);
        activestorage = false;
        Intake.stop();
        timer.reset();
    }

    public static Status getState() {
        if (System.currentTimeMillis() - last > updateTime) {
            if (bdown.getBeamState().equals(BreakBeam.Status.CLEAR) && up.getState())
                status = Status.ONEBALL;
            else if (bdown.getBeamState().equals(BreakBeam.Status.BROKEN)&& up.getState())
                status = Status.STORAGEFULL;
            else if (bdown.getBeamState().equals(BreakBeam.Status.CLEAR) && !up.getState())
                status = Status.EMPTY;

            last = System.currentTimeMillis();
        }
        return status;
    }

    public static void StorageForcedStop() {
        if (getState().equals(Status.STORAGEFULL)) stop();
    }

    public static void checkUpLaser() {
        if (up.getState()) stop();
    }


    public static void update(){
        getState();
        if ((status.equals(Status.ONEBALL) || status.equals(Status.STORAGEFULL)) && !Shooter.readyForShoot) stop();
    }

    public static int count = 0;

    public static void update2() {
        if(bdown.getBeamState().equals(BreakBeam.Status.BROKEN) && System.currentTimeMillis() - last > updateTime){
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
        if(count >= 1 && !Shooter.readyForShoot) stop();
    }
}