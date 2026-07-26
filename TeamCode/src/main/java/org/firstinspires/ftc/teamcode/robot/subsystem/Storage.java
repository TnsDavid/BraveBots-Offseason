package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;
import org.firstinspires.ftc.teamcode.util.wrappers.RGBIndicator;

@Configurable
public class Storage {
    public static double ultraClosePower =0.961, closePower =0.85, farPower = 0.53;
    public static BetterMotorEx transfer;
    public static RGBIndicator ledST,ledDR;
    public static BreakBeam bUp, bDown;
    public static DigitalChannel laserMiddle;
    public static double last, updateTime = 100,NONE=0;

    public static RGBIndicator.Colors colour = RGBIndicator.Colors.WHITE;
    public enum Status {
        EMPTY, ONEBALL, TWOBALL, STORAGEFULL

    }
    public static boolean isEmpty()
    {
        return status==Status.EMPTY;
    }
    public enum ActiveStatus{
        TRANSFER, NOTRANSFER
    }
    public static Status status = Status.EMPTY;

    public static double startPower = closePower;
    public static ActiveStatus activeStatus = ActiveStatus.NOTRANSFER;
    public static void start() {
        activeStatus = ActiveStatus.TRANSFER;
    }
    public static void lowerPower() {
        transfer.setPower(0.65);
        activeStatus = ActiveStatus.TRANSFER;
    }
    public static void stop() {
        transfer.setPower(0);
        activeStatus = ActiveStatus.NOTRANSFER;
    }
    public static int count = 0;
    public static ElapsedTime ballUp;

    public static void setColour(RGBIndicator.Colors colour){
        ledDR.setColor(colour);
        ledST.setColor(colour);
    }
    public static boolean up = false, down = false, middle = false;
    public static boolean startedTimer = false;
    public static double storageStopTime = 60;

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
                colour= RGBIndicator.Colors.WHITE;
                break;
            case 1:
                status = Status.ONEBALL;
                colour= RGBIndicator.Colors.RED;
                break;
            case 2:
                status = Status.TWOBALL;
                colour= RGBIndicator.Colors.PURPLE;
                break;

            case 3:
                if(bDown.isBeamBroken()&& bUp.isBeamBroken()) {
                    status = Status.STORAGEFULL;
                    colour= RGBIndicator.Colors.GREEN;
                }
                else if(!bUp.isBeamBroken()) {
                    status = Status.ONEBALL;
                    count = 1;
                    colour= RGBIndicator.Colors.RED;
                } else {
                    status = Status.TWOBALL;
                    count = 2;
                    colour= RGBIndicator.Colors.BLUE;
                }
                break;
        }

        setColour(colour);

    }
    public static boolean started=false;
    public static int valids = 0;
    public static void update2(){
        if(!Shooter.readyForShoot){
            if(!startedTimer && bUp.isBeamBroken()){
                ballUp = new ElapsedTime();
                startedTimer = true;
            }
            if(ballUp != null && ballUp.milliseconds()>storageStopTime && startedTimer) stop();
        }
        up = bUp.isBeamBroken();
        down = bDown.isBeamBroken();
        middle = laserMiddle.getState();

        if( !up && !middle && !down ) valids = 0;
        else if( up && !middle && !down ) valids = 1;
        else if( up && middle && !down ) valids = 2;
        else if( up && middle && down ) valids = 3;

        switch(valids){
            case 0:
                status = Status.EMPTY;
                colour = RGBIndicator.Colors.WHITE ;
                break;
            case 1:
                status = Status.ONEBALL;
                colour= RGBIndicator.Colors.RED;
                break;
            case 2:
                status = Status.TWOBALL;
                colour = RGBIndicator.Colors.PURPLE;
                break;
            case 3:
                status = Status.STORAGEFULL;
                colour = RGBIndicator.Colors.GREEN;
                break;
        }
        setColour(colour);
        if(Shooter.readyForShoot) {
            if (Robot.playStyle == Robot.PlayStyle.CLOSE) {
                if (Turret.virtualDist <= 2.225) startPower = ultraClosePower;
                else startPower = closePower;
            } else startPower = farPower;
        } else startPower = 1;
        double currentVoltage=Robot.voltageSensor.getVoltage();
        startPower=startPower*(Robot.nominalVoltage/currentVoltage);
        if(activeStatus==ActiveStatus.TRANSFER) transfer.setPower(startPower);
        else transfer.setPower(0);

    }


    public static void updateAuto(){
        if(!Shooter.readyForShoot){
            if(!startedTimer && bUp.isBeamBroken()){
                ballUp = new ElapsedTime();
                startedTimer = true;
            }
            if(ballUp != null && ballUp.milliseconds()>storageStopTime && startedTimer) stop();
        }
        up = bUp.isBeamBroken();
        down = bDown.isBeamBroken();
        middle = laserMiddle.getState();

        if( !up && !middle && !down ) valids = 0;
        else if( up && !middle && !down ) valids = 1;
        else if( up && middle && !down ) valids = 2;
        else if( up && middle && down ) valids = 3;

        switch(valids){
            case 0:
                status = Status.EMPTY;
                colour = RGBIndicator.Colors.WHITE ;
                break;
            case 1:
                status = Status.ONEBALL;
                colour= RGBIndicator.Colors.RED;
                break;
            case 2:
                status = Status.TWOBALL;
                colour = RGBIndicator.Colors.PURPLE;
                break;
            case 3:
                status = Status.STORAGEFULL;
                colour = RGBIndicator.Colors.GREEN;
                break;
        }
        setColour(colour);
        if(Shooter.readyForShoot) {
            startPower = farPower;
        } else startPower = 1;
        double currentVoltage=Robot.voltageSensor.getVoltage();
        startPower=startPower*(Robot.nominalVoltage/currentVoltage);
        if(activeStatus==ActiveStatus.TRANSFER) transfer.setPower(startPower);
        else transfer.setPower(0);
    }


}