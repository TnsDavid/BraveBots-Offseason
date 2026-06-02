package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.PIDController;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

public class Shooter {
    public static BetterMotorEx motor1, motor2;
    public static BetterServo servo,SE;
    public static double downPos= 0.013;
    public static boolean shooting= false;

    public static final double a1 =-0.0697152, b1 = 0.410972, c1 = -0.401803;
    public static final double a2 =-9.19567, b2 =229.70232, c2 = 504.01862;
    public static double p= -0.004,i,d,f = 1;
    public static double velocity = 0.0;
    static PIDFCoefficients shootercoeffs = new PIDFCoefficients(p, i, d, f);

    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(fS,fV,fA);
    public PIDController veloController = new PIDController(vP,vI,vD);
    //  public static double vP=5,vI=0,vD=0.1,fS=200,fV=1.5,fA=0;
    public static double vP=5.3,vI=0,vD=0.1,fS=200,fV=1.68,fA=0;
    public static void setCoefs(){
        motor1.setPFCoefficients(p,f);
        motor2.setPFCoefficients(p,f);
    }
    public static double voltageThreshold = 11.1;
    public static void setVelocity(double vel){
        double voltage = Robot.voltageSensor.getVoltage();
        double coeff = (voltage < voltageThreshold) ? voltageThreshold / voltage : 1;
        vel *= coeff;
        motor1.setVelocity(vel);
        motor2.setPower(motor1.getPower());
    }
    public static boolean wereCoeffsSet = false;
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
    public static double hoodPos;
    public static double est;
    public static double offset=0;
    public static double boost;
    public static void update(){
        double distt= Turret.dist;

        if(distt< 3.3) {

//        if(distt<3) {
            velocity = Range.clip(a2 * Math.pow(distt, 2) + b2 * distt + c2 + offset, 0, 2100);
            hoodPos = Range.clip(a1*Math.pow(distt,2)+ b1*distt+c1, 0.09,0.3172);
        }
        else {
            velocity = 1220+offset;
            hoodPos = 0.2011;
        }

//        else {
//            velocity= 1220+offset;
//            hoodPos= 0.2131;
//        }
            setVelocity(velocity);
            setVelocity(0);
        servo.setPosition(hoodPos);
        motor1.update();
    }
}
