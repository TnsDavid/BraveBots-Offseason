package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.math.PDSFController;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

@Configurable
public class Shooter {
    public static BetterMotorEx motor1, motor2;
    public static double velocity = 0;
    public static BetterServo hoodServo;
    public static boolean readyForShoot = false;
    public static double pClose = 0.005, fClose = 1.15, s = 0.1, d = 0;
    public static double alpha = 0.5;


    public static PDSFController controller = new PDSFController(pClose, 0, s, fClose / 1700);

    public static LowPassFilter ShooterLpf, HoodLpf;

    public static void setLpfAlpha() {
        ShooterLpf.setAlpha(alpha);
    }

    public static void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public static double hoodPos = 0, minHoodpos = 0.3322, maxHoodPos = 0.8506, velocityFAR = 1100, closeInc = 0.0082;
    public static double hoodFAR = 0.6272;
    public static double offset = 0, velocityThreshold = 60;
    public static boolean shooterEnabled = false;
    public static double hoodCompensation, naturalPos;

    public static double calculateHoodByError(double pos, double velocity) {
        hoodCompensation = HoodLpf.update(pos * (motor1.getVelocity() / velocity));
        return Range.clip(hoodCompensation, minHoodpos, maxHoodPos);
    }

    //1120 - auto

    //3.94 1190
    //3.29 1120
    //3.14 1100

    public static double error = 0;
    public static double theta = 0.8;
    public static void update() {
        HoodLpf.setAlpha(theta);
        controller.setCoefficients(pClose, 0, fClose / 1700, s);
        if (Robot.playStyle == Robot.PlayStyle.CLOSE) {
            velocity = ShooterMath.calculateVelocityFromDistance(Turret.virtualDist) + offset;
            hoodPos = !Shooter.readyForShoot ? ShooterMath.calculateHoodAngleFromDistance(Turret.virtualDist) : ShooterMath.calculateHoodAngleFromVelocity(motor1.getVelocity());
        } else {
            velocity = ShooterMath.calculateVelocityFARFromDistance(Turret.virtualDist) + offset;
            hoodPos = ShooterMath.calculateHoodFARFromDistance(Turret.virtualDist);
            hoodPos = calculateHoodByError(hoodPos, velocity);
        }
        hoodServo.setPosition(hoodPos);
        double currentVoltage = Robot.voltageSensor.getVoltage();
        double pid = controller.calculate(motor1.getVelocity(), velocity);
        double output = pid * (Robot.nominalVoltage / currentVoltage);
        if (shooterEnabled) setPower(output);
        else setPower(0);
    }

    public static void AutoUpdate(double velocity, double hoodPos) {
        Shooter.velocity = velocity;
        if (Shooter.readyForShoot) hoodPos = calculateHoodByError(hoodPos, velocity);
        double currentvoltage = Robot.voltageSensor.getVoltage();
        double pid = controller.calculate(motor1.getVelocity(), velocity);
        double output = pid * (Robot.nominalVoltage / currentvoltage);
        hoodServo.setPosition(hoodPos);
        setPower(output);
    }

    public static void tuningupdate(double velocity) {
        controller.setCoefficients(pClose, 0, fClose / 1700, s);
        Shooter.velocity = velocity;
        double currentvoltage = Robot.voltageSensor.getVoltage();
        double pid = controller.calculate(motor1.getVelocity(), velocity);
        double output = pid * (Robot.nominalVoltage / currentvoltage);
        if (shooterEnabled) setPower(output);
        else setPower(0);
    }
}