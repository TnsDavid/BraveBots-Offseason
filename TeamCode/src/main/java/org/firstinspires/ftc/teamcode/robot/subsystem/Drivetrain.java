package org.firstinspires.ftc.teamcode.robot.subsystem;

import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Drivetrain {
    public static BetterMotorEx leftFront,leftBack,rightBack,rightFront;
    public static void drive(double x, double y , double turn){
        double theta = Math.atan2(y,x);
        double power = Math.hypot(x,y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double lf = power * cos/max +turn;
        double rf = power * sin/max -turn;
        double lb = power * sin/max +turn;
        double rb = power * cos/max -turn;

        if((power + Math.abs(turn)) > 1){
            lf /= power+turn;
            rf/=power+turn;
            rb/=power+turn;
            lb/=power+turn;
        }


        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);

    }

    public static double climbPower = 1;
    public static void climb(){
        leftFront.setPower(climbPower);
        rightFront.setPower(climbPower);
        leftBack.setPower(climbPower);
        rightBack.setPower(climbPower);
    }
}
