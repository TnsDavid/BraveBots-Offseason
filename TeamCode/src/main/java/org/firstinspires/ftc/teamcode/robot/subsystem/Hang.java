package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterCRServo;

import java.util.Timer;

public class Hang {
    public static BetterCRServo[] hangServosLeft = new BetterCRServo[3];
    public static BetterCRServo[] hangServosRight = new BetterCRServo[3];

    public static boolean raised = false;
    public static void hang() {
        for (int i = 0; i < 3; i++) {
            hangServosLeft[i].setPower(1);
            hangServosRight[i].setPower(1);
        }
    }

    public static void stopHang() {
        for (int i = 0; i < 3; i++) {
            hangServosLeft[i].setPower(0);
            hangServosRight[i].setPower(0);
        }
    }

    public static void setPower(double trigger){
        for(int i = 0; i < 3; i++) {
            hangServosLeft[i].setPower(trigger);
            hangServosRight[i].setPower(trigger);
        }
    }
    public static void update(ElapsedTime timer) {
        double power = timer.seconds() < 3 ? 1 : 0;
        setPower(power);
    }
}
