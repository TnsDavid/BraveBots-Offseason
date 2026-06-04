package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Intake {
    public static BetterMotorEx motor;
    static ElapsedTime timer;
    public static boolean intaking = false;
    public static void start(){
        motor.setPower(.94);
        timer= new ElapsedTime();
        intaking= true;
    }
    public static void relanti(){
        motor.setPower(0.5);
        intaking = true;
    }
    public static void stop(){
        motor.setPower(0);
        intaking= false;
    }
    public static void reverse(){
        motor.setPower(-0.5);
        intaking = false;
    }
    public static void setPower(double power){
        motor.setPower(power);
    }
    public static void toggle(){
        if(!intaking){
            start();
            Storage.start();
        }
        else{
            stop();
            Storage.stop();
        }
    }
}