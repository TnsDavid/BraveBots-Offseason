package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

public class Blocker {

    public static BetterServo blocker;
    public static double locked =0.5472 ,  unlocked = 0.6578;

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
    public static boolean atPosition() {
        double targetPosition = blocked ? locked : unlocked;
        return Math.abs(targetPosition - blocker.getPosition()) < 0.01;
    }
}