package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.Test;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.PIDController;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

@Configurable
public class Shooter {
    public static BetterMotorEx motor1, motor2;
    public static double velocity=0;
    public static BetterServo servo;
    public static boolean readyForShoot = false;


    //public static final double aVClose = -56.69335, bVClose = 587.26292, cVClose = 748.70738;

    //public static final double aHClose = -0.0756773, bHClose = 0.53089, cHClose = 0.0499127 ;

    public static double pClose= -0.0075,fClose = 0.77;
    //public static double pFar = -0.004 , fFar = 0.9, s = 0.1;
    public static double pFar = -0.0075 , fFar = 0.77, s = 0.1;

    public static void setCoefs() {
        double pFinal , fFinal ;
        pFinal = Robot.playStyle == Robot.PlayStyle.CLOSE ? pClose : pFar;
        fFinal = Robot.playStyle == Robot.PlayStyle.CLOSE ? fClose : fFar;
        motor1.setPSFCoefficients(pFinal,fFinal , s);
        motor2.setPSFCoefficients(pFinal,fFinal, s);
    }
    public static void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public static double voltageThreshold = 10.5;
    public static void setVelocity(double vel){
        double voltage = Robot.voltageSensor.getVoltage();
        double coeff = (voltage < voltageThreshold) ? voltageThreshold / voltage : 1;
        vel *= coeff;
        motor1.setVelocity(vel);
        motor2.setPower(motor1.getPower());
    }
    public static double hoodPos = 0, minHoodpos = 0.45 ,maxHoodPos = 0.96,velocityFAR=2000,hoodFAR=0.93;
    public static double offset=0;
    public static boolean shooterEnabled = false;
    public static void update() {
        if (shooterEnabled){
            double virtualDistt = Turret.virtualDist;
            if(Robot.playStyle == Robot.PlayStyle.CLOSE) {
                velocity = ShooterMath.calculateVelocity(virtualDistt);
                hoodPos = ShooterMath.calculateHoodAngle(virtualDistt);
            }
            else {
                velocity=velocityFAR;
                hoodPos=hoodFAR;
            }
            if(!Shooter.readyForShoot || Turret.sotm)
                servo.setPosition(hoodPos);
            setVelocity(velocity);
            motor1.update();
        } else setPower(0);
    }
}