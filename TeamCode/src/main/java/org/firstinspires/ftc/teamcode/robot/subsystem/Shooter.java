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
    public static BetterServo servo,SE;
    public static double downPos= 0.013;
    public static boolean readyForShoot = false;


    //y=-56.69335x^{2}+587.26292x+748.70738
    public static final double aVClose = -56.69335, bVClose = 587.26292, cVClose = 748.70738;
    //y=-0.00353031x^{2}+0.283927x+0.257332
    public static final double aHClose = -0.00353031 , bHClose = 0.283927, cHClose = 0.257332;
    public static double p= 0.005,i,d,f = 0.74;
    public static double pCoef=-5;
    public static double velDiff = 200;
    //public static double velocity = 0.0;
    static PIDFCoefficients shootercoeffs = new PIDFCoefficients(p, i, d, f);
    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(fS,fV,fA);
    public PIDController veloController = new PIDController(vP,vI,vD);
    //  public static double vP=5,vI=0,vD=0.1,fS=200,fV=1.5,fA=0;
    public static double vP=5.3,vI=0,vD=0.1,fS=200,fV=1.68,fA=0;
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
    public static double hoodPos = 0, minHoodpos = 0.42 ,maxHoodPos = 0.96;
    public static double est;
    public static double offset=0;
    public static void updateTest(){
        motor1.update();
        servo.setPosition(hoodPos);
    }
    public static double boost;
    public static void update(){
        double distt= Turret.dist;
        double virtualDistt = Turret.virtualDist;

        if( Test.shooterEnabled ) {
            velocity = aVClose * Math.pow(distt, 2) + bVClose * distt + cVClose;
            hoodPos = Range.clip(aHClose * Math.pow(distt, 2) + bHClose * distt + cHClose, minHoodpos, maxHoodPos);
        }

        setVelocity(velocity);
        servo.setPosition(hoodPos);
        motor1.update();
    }
}