package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Odo;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Drivetrain {
    private static final Logger log = LoggerFactory.getLogger(Drivetrain.class);
    public static BetterMotorEx leftFront,leftBack,rightBack,rightFront;

    public static double[] teleopDriveValues ;
    public static Vector teleopDriveVector = new Vector() , teleopHeadingVector = new Vector() ;
    public static void drive(double x, double y, double rx){
        x*= 1.1;

        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double fl = (y + x + rx) / d;
        double bl = (y - x + rx) / d;
        double fr = (y - x - rx) / d;
        double br = (y + x - rx) / d;

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightBack.setPower(br);
        rightFront.setPower(fr);
    }
    public static void setTeleopDrive(double x , double y , double turn){
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
    public static void toggleControlMethod() {
        robotCentric = !robotCentric;
    }

    public static void betterDrive(double rawX, double rawY, double rotation) {
        double finalX, finalY;
        double influentaRotatiei = Robot.alliance == Alliance.RED ? 1 : -1;
        if( !robotCentric ) {
            double robotHeading = Odo.odo.getHeading(AngleUnit.RADIANS);
            finalX = (rawX * Math.cos(-robotHeading) - rawY * Math.sin(-robotHeading)) * influentaRotatiei;
            finalY = (rawX * Math.sin(-robotHeading) + rawY * Math.cos(-robotHeading)) * influentaRotatiei;
        }
        else {
            finalX = rawX;
            finalY = rawY;
        }

        drive(finalX, finalY, rotation);

    }


}
