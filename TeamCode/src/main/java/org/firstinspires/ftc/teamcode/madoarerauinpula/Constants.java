package org.firstinspires.ftc.teamcode.madoarerauinpula;

import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.control.PredictiveBrakingController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.math.PIDFController;

public class Constants {
    public static double xOffset = 0 ;
    public static double yOffset = 0 ;

    public static double headingConstraint = 0.1;
    public static double headingVelocityConstraint = 0.1;
    public static double XYVelocityConstraint= 1;;

    public static double p = 0;
    public static double linearbraking = 0;
    public static double kquadratric = 0;

    public static double pforward , pstrafe , pheading , dstrafe , dforward , dheading , fstrafe , fforward , fheading;
    public static PIDFController forward = new PIDFController(new PIDFCoefficients(pforward , 0 , dforward , fforward));
    public static PIDFController strafe = new PIDFController(new PIDFCoefficients(pstrafe , 0 , dstrafe , fstrafe));
    public static PIDFController heading = new PIDFController(new PIDFCoefficients(pheading , 0 , dheading , fheading));

    public static PredictiveBrakingController predictiveController = new PredictiveBrakingController(new PredictiveBrakingCoefficients(p ,linearbraking , kquadratric));

}
