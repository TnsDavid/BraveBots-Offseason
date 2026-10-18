package org.firstinspires.ftc.teamcode.madoarerauinpula.PathPlanning;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.math.Vector;


public class Pose2D {
    public double x,y,heading, defaultTolerance;
    DistanceUnit unit;
    AngleUnit angleunit;
    public enum CoordinateSystem{
        CARTESIAN, POLAR;
    }

    CoordinateSystem coordinateSystem ;
    public Pose2D(double x, double y , double heading , double defaultTolerance, AngleUnit angleunit , DistanceUnit unit){
        coordinateSystem = CoordinateSystem.CARTESIAN;
        this.x = x;
        this.y = y;
        this.heading =heading;
        this.defaultTolerance = defaultTolerance;
        this.unit = unit;
        this.angleunit = angleunit;
    }

    public Pose2D(double x, double y ,double heading , double defaultTolerance){
        this(x , y , heading , defaultTolerance, AngleUnit.RADIANS , DistanceUnit.INCH);
    }
    public Pose2D(double x, double y ,double heading){
        this(x , y , heading , 0.5 , AngleUnit.RADIANS , DistanceUnit.INCH);
    }
    public Pose2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }
    public double getHeading(){
        return heading;
    }
    public double getY(){
        return y;
    }
    public double getDefaultTolerance(){
        return defaultTolerance;
    }
    public Pose2D getPose(){
        return this;
    }

    public Vector getAsVector() {
        return new Vector(x , y);
    }

        public Vector convertToPolar(){
        coordinateSystem = CoordinateSystem.POLAR;
        return new Vector(this.getAsVector().getMagnitude() , this.getAsVector().getTheta() , Vector.Type.POLAR);
    }
    public double getDistance(Pose2D other){
        return Math.sqrt((other.getX() - x)*(other.getX() - x) + (other.getY() - y)*(other.getY() - y));
    }

    public boolean isReached(Pose2D other){
        return getDistance(other) <= defaultTolerance;
    }

    public Pose2D mirrorOnYAxis(){
        Pose2D p= this;
        double x, heading;
        x= 144- p.getX();
        heading = p.getHeading();
        while (heading > 2* Math.PI)
            heading-= 2 * Math.PI;
        while (heading< 0)
            heading+= 2* Math.PI;

        heading= heading<= Math.PI ? Math.PI- heading : 3 * Math.PI - heading;

        return new Pose2D(x, p.getY(), heading);
    }
}
