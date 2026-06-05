package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

public class Blocker {

    public static BetterServo blocker;
    public static double locked =0 ,  unlocked = .7;

    public static boolean blocked = true;

    public static void unblock() {
        blocker.setPosition(unlocked);
        blocked = false;
    }
    public static void block() {
        blocker.setPosition(locked);
        blocked = true;
    }

    public static void toggle(){
        if(blocked) unblock();
        else block();
    }
}