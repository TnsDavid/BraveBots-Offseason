package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Globals;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Drivetrain {
    private static final Logger log = LoggerFactory.getLogger(Drivetrain.class);
    public static BetterMotorEx leftFront,leftBack,rightBack,rightFront;
    public static double powerLimit = 1;
    public static double[] teleopDriveValues ;
    public static Vector teleopDriveVector = new Vector() , teleopHeadingVector = new Vector() ;
    public static void drive(double x, double y, double rx){
        x*= 1.2;

        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double fl = (y + x + rx) / d;
        double bl = (y - x + rx) / d;
        double fr = (y - x - rx) / d;
        double br = (y + x - rx) / d;

        if(Globals.playingDefense ) {
            if (Robot.voltage * (80 / 3.3) > 30) powerLimit = 0.5;
            else powerLimit = 1;
        } else powerLimit = 1;

        fl *= powerLimit;
        bl *= powerLimit;
        fr *= powerLimit;
        br *= powerLimit;

        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightBack.setPower(br);
        rightFront.setPower(fr);
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
