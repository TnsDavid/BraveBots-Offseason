package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tests.TestFCDrivetrain;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Drivetrain {
    public static BetterMotorEx leftFront,leftBack,rightBack,rightFront;
    //public static boolean robotCentric = false;
    public static GoBildaPinpointDriver pinpoint;
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
    public static boolean robotCentric = true;
    public static void betterDrive(double rawX, double rawY, double rotation) {
        double finalX, finalY;
        if( !robotCentric ) {
            double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
            finalX = rawX * Math.cos(-robotHeading) - rawY * Math.sin(-robotHeading);
            finalY = rawX * Math.sin(-robotHeading) + rawY * Math.cos(-robotHeading);
        } else {
            finalX = rawX;
            finalY = rawY;
        }

        drive(finalX, finalY, rotation);

    }
}
