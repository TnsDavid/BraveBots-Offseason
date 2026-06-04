package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

public class Blocker {

    public static BetterServo blocker;
    public static double locked = 0, unlocked = 0;

    public static void unblock() {
        blocker.setPosition(unlocked);
    }
    public static void block() {
        blocker.setPosition(locked);
    }
}