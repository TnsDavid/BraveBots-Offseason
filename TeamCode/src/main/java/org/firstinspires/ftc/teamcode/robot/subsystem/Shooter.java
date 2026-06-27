package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.PDSFCoefficients;
import org.firstinspires.ftc.teamcode.util.math.PDSFController;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

@Configurable
public class Shooter {
    public static BetterMotorEx motor1, motor2;
    public static double velocity=0;
    public static BetterServo hoodServo;
    public static boolean readyForShoot = false;
    public static double pClose= 0.0075,fClose = 0.77,s=0.1;
    //public static double pFar = -0.004 , fFar = 0.9, s = 0.1;
    //public static double pFar = 0.0075 , fFar = 0.77, s = 0.1;


   public static PDSFController controller = new PDSFController(pClose , 0 , s , fClose/2600);


    public static void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public static double hoodPos = 0, minHoodpos = 0.45 ,maxHoodPos = 0.96,velocityFAR=1960, closeInc = 0.0082;
    //public static double hoodFAR=0.937;
    public static double hoodFAR=0.945;
          //  public static double hoodFAR=0.8494;
    public static double offset=0, velocityThreshold = 60;
    public static boolean shooterEnabled = false;




    public static void update() {
        if (shooterEnabled){
            double virtualDistt = Turret.virtualDist;
            if(Robot.playStyle == Robot.PlayStyle.CLOSE) {
                velocity = ShooterMath.calculateVelocity(Turret.virtualDist);
                hoodPos = ShooterMath.calculateHoodAngle(Turret.virtualDist) + 0.0082;
            }
            else {
                velocity=velocityFAR;
                hoodPos=hoodFAR;
            }
            if(!Shooter.readyForShoot || Turret.sotm)
                hoodServo.setPosition(hoodPos);
            double currentvoltage = Robot.voltageSensor.getVoltage();
            double pid=controller.calculate(motor1.getVelocity(),velocity);
            double output=pid*(Robot.nominalVoltage/currentvoltage);
            setPower(output);
        } else setPower(0);
    }
    public static void updateAuto() {
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
                hoodServo.setPosition(hoodPos);
            double currentvoltage = Robot.voltageSensor.getVoltage();

            double pid=controller.calculate(motor1.getVelocity(),velocity);
            double output=pid*(Robot.nominalVoltage/currentvoltage);
            setPower(output);
        } else setPower(0);
    }
    public static void tuningUpdate(double velocity) {
        Shooter.velocity=velocity;
        double currentvoltage = Robot.voltageSensor.getVoltage();
        double pid=controller.calculate(motor1.getVelocity(),velocity);
        double output=pid*(Robot.nominalVoltage/currentvoltage);
        setPower(output);
    }
}