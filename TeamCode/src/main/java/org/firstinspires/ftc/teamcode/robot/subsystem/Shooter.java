package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.LowPassFilter;
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
    public static double pClose= 0.008,fClose = 1.125,s=0.1;
    public static double alpha = 0.5;
    //public static double pFar = -0.004 , fFar = 0.9, s = 0.1;
    //public static double pFar = 0.0075 , fFar = 0.77, s = 0.1;


    public static PDSFController controller = new PDSFController(pClose , 0 , s , fClose/2600);

    public static LowPassFilter ShooterLpf;

    public static void setLpfAlpha(){
        ShooterLpf.setAlpha(alpha);
    }

    public static void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public static double hoodPos = 0, minHoodpos = 0.45 ,maxHoodPos = 0.96,velocityFAR=1960, closeInc = 0.0082;
    //public static double hoodFAR=0.937;
    public static double hoodFAR=0.945 , hoodFarAuto = 0.92;
          //  public static double hoodFAR=0.8494;
    public static double offset=0, velocityThreshold = 60;
    public static boolean shooterEnabled = false;




    public static void update() {
        if (shooterEnabled){
            if (Robot.playStyle == Robot.PlayStyle.CLOSE) {
                velocity = ShooterMath.calculateVelocityFromDistance(Turret.virtualDist) + offset;
                hoodPos = !Shooter.readyForShoot ? ShooterMath.calculateHoodAngleFromDistance(Turret.virtualDist) : ShooterMath.calculateHoodAngleFromVelocity(motor1.getVelocity());
            } else {
                velocity = ShooterMath.calculateVelocityFARFromDistance(Turret.virtualDist) + offset;
                hoodPos = !Shooter.readyForShoot ? hoodFAR : hoodFAR * (motor1.getVelocity() / velocityFAR);
            }
            hoodServo.setPosition(hoodPos);
            double currentVoltage = Robot.voltageSensor.getVoltage();
            double pid=controller.calculate((motor1.getVelocity()),velocity);
            double output=pid*(Robot.nominalVoltage/currentVoltage);
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