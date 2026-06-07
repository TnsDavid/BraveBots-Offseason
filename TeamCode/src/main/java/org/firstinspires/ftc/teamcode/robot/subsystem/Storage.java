package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.Test;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;

public class Storage {
    public static BetterMotorEx transfer;
    public static DigitalChannel up;
    public static BreakBeam bup, bmiddle, bdown;
    public static boolean activestorage;
    public static ElapsedTime timer;
    public static double last, minimumUpdateTime = 300;


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
        if (System.currentTimeMillis() - last > minimumUpdateTime) {
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

    public static double count = 0;

    public static void update(){
        getState();
        if (status.equals(Status.ONEBALL)) stop();
        else if(Shooter.readyForShoot) start();
    }
}