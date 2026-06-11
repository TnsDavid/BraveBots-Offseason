package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.tests.TestFCDrivetrain;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

public class Drivetrain {
    public static BetterMotorEx leftFront,leftBack,rightBack,rightFront;
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
        if( !robotCentric ) {
            double robotHeading = Robot.odo.getHeading(AngleUnit.RADIANS);
            finalX = rawX * Math.cos(-robotHeading) - rawY * Math.sin(-robotHeading);
            finalY = rawX * Math.sin(-robotHeading) + rawY * Math.cos(-robotHeading);
        } else {
            finalX = rawX;
            finalY = rawY;
        }

        drive(finalX, finalY, rotation);

    }
}
