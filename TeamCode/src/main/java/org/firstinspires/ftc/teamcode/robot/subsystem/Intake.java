package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Intake {
    public static BetterMotorEx motor;
    static ElapsedTime timer = new ElapsedTime();

    public static IntakeStatus status = IntakeStatus.NOTINTAKING;

    public enum IntakeStatus{
        INTAKING, NOTINTAKING
    }
    public static void start(){
        motor.setPower(.94);
        timer= new ElapsedTime();
        status= IntakeStatus.INTAKING;
    }
    public static void relanti(){
        motor.setPower(0.5);
        status = IntakeStatus.INTAKING;
    }
    public static void stop(){
        motor.setPower(0);
        status= IntakeStatus.NOTINTAKING;
    }
    public static void reverse(){
        motor.setPower(-0.5);
        status = IntakeStatus.NOTINTAKING;
    }
    public static void setPower(double power){
        motor.setPower(power);
    }
    public static void toggle(){
        if(status == IntakeStatus.NOTINTAKING){
            if(!(Storage.status.equals(Storage.Status.STORAGEFULL) && Storage.bUp.isBeamBroken())) Intake.start();
            if(!Storage.bUp.isBeamBroken()) Storage.start();
        }
        else{
            stop();
            Storage.stop();
        }
    }
    public static void update() {
        if(Storage.status.equals(Storage.Status.STORAGEFULL) && !Shooter.readyForShoot) stop();
    }
}