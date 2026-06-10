package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.Test;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.PIDController;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

@Configurable
public class Shooter {
    public static BetterMotorEx motor1, motor2;
    public static double velocity=0;
    public static BetterServo servo;
    public static boolean readyForShoot = false;


    public static final double aVClose = -56.69335, bVClose = 587.26292, cVClose = 748.70738;
    //y=-0.0756773x^{2}+0.53089x+0.0499127
    public static final double aHClose = -0.0756773, bHClose = 0.53089, cHClose = 0.0499127 ;
    public static double p= 0.005,f = 0.74;
    public static void setCoefs() {
        motor1.setPFCoefficients(p,f);
        motor2.setPFCoefficients(p,f);
    }
    public static double voltageThreshold = 10.5;
    public static void setVelocity(double vel){
        double voltage = Robot.voltageSensor.getVoltage();
        double coeff = (voltage < voltageThreshold) ? voltageThreshold / voltage : 1;
        vel *= coeff;
        motor1.setVelocity(vel);
        motor2.setPower(motor1.getPower());
    }
    public static boolean wereCoeffsSet = false;
     public static void setReady() {
        readyForShoot = !readyForShoot;
    }
    public static void setVelocity2(double vel){
        if(!wereCoeffsSet)
        {
            wereCoeffsSet = true;
            setCoefs();
        }
        try {
            motor1.setVelocity(vel);
            motor2.setVelocity(vel);
        }
        catch (TargetPositionNotSetException ignored){

        }
    }
    public static double hoodPos = 0, minHoodpos = 0.45 ,maxHoodPos = 0.96;
    public static double offset=0;
    public static void updateTest(){
        motor1.update();
        servo.setPosition(hoodPos);
    }
    public static void update() {
        double distt = Turret.dist;
        double virtualDistt = Turret.virtualDist;

        if (Test.shooterEnabled){
            if(Robot.playStyle == Robot.PlayStyle.CLOSE) {
                velocity = aVClose * Math.pow(virtualDistt, 2) + bVClose * virtualDistt + cVClose;
                hoodPos = Range.clip(aHClose * Math.pow(virtualDistt, 2) + bHClose * virtualDistt + cHClose, minHoodpos, maxHoodPos);
            }
            else{
                velocity=2100;
                hoodPos=0.9072;
            }
        }

        setVelocity(velocity);
        if(!readyForShoot) {
            servo.setPosition(hoodPos);
        }
        motor1.update();
    }
}