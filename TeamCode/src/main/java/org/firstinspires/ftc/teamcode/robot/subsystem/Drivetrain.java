package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.tests.TestFCDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.Vector2D;
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
        double influentaRotatiei = Robot.a == Alliance.RED ? 1 : -1;
        if( !robotCentric ) {
            double robotHeading = Robot.odo.getHeading(AngleUnit.RADIANS);
            finalX = (rawX * Math.cos(-robotHeading) - rawY * Math.sin(-robotHeading)) *  influentaRotatiei;
            finalY = (rawX * Math.sin(-robotHeading) + rawY * Math.cos(-robotHeading)) * influentaRotatiei;
        }
        else {
            finalX = rawX;
            finalY = rawY;
        }

        drive(finalX, finalY, rotation);

    }
    public static void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric, double headingOffset) {
        try {
            teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
            teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
            teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
            teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
            teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

            if (robotCentric) {
                teleopDriveVector.rotateVector(Robot.robotPose.getHeading());
            }

            teleopDriveVector.rotateVector(headingOffset);

            teleopHeadingVector.setComponents(teleopDriveValues[2], Robot.robotPose.getHeading());
        } catch (Exception e) {
            log.error("e: ", e);
        }
    }
}
