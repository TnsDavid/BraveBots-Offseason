package org.firstinspires.ftc.teamcode.madoarerauinpula;



import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.math.PIDFController;

public class Constants {
    public static double xOffset = -4.02, yOffset = 0.944;

    public static double headingConstraint = 0.1;
    public static double headingVelocityConstraint = 0.1;
    public static double XYVelocityConstraint= 1;;

    public static double p = 0;


    /// Aici pui valorile din deceleration tuner
    public static double xDeceleration = 23.0019;  // 55.379
    public static double yDeceleration = 33.3739;

    public static double pforward , pstrafe , pheading , dstrafe , dforward , dheading , fstrafe , fforward , fheading;
    public static PIDFController forward = new PIDFController(new PIDFCoefficients(pforward , 0 , dforward , fforward));
    public static PIDFController strafe = new PIDFController(new PIDFCoefficients(pstrafe , 0 , dstrafe , fstrafe));
    public static PIDFController heading = new PIDFController(new PIDFCoefficients(pheading , 0 , dheading , fheading));


}
