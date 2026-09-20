package org.firstinspires.ftc.teamcode.madoarerauinpula;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.madoarerauinpula.PathPlanning.Pose2D;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.util.math.LowPassFilter;

public class Localizer {
    public Pose2D currentPose;

    public double x , y , predictedX, predictedY , heading;
    public Localizer(){
      currentPose = new Pose2D(Odo.odo.getPosX(DistanceUnit.INCH) , Odo.odo.getPosY(DistanceUnit.INCH) , Odo.odo.getHeading(AngleUnit.RADIANS));
    }

    public double getPosX(DistanceUnit distanceUnit){
        return Odo.odo.getPosX(distanceUnit);
    }
    public double getPosY(DistanceUnit distanceUnit){
        return Odo.odo.getPosY(distanceUnit);
    }
     public double getHeading(AngleUnit angleUnit){
        return Odo.odo.getHeading(angleUnit);
    }

    public void setStartingPose(Pose2D startingPose){
        Odo.odo.setPosX(startingPose.getX() , DistanceUnit.INCH);
        Odo.odo.setPosY(startingPose.getY() , DistanceUnit.INCH);
        Odo.odo.setHeading(startingPose.getHeading() , AngleUnit.RADIANS);
    }
    public void setPose(Pose2D pose){
        Odo.odo.setPosX(pose.getX() , DistanceUnit.INCH);
        Odo.odo.setPosY(pose.getY() , DistanceUnit.INCH);
        Odo.odo.setHeading(pose.getHeading() , AngleUnit.RADIANS);
    }
    public double getPredictedX(){
        return predictedX;
    }
    public double getPredictedY(){
        return predictedY;
    }


    public Pose2D getPredictedPose(){
        return new Pose2D(getPredictedX() , getPredictedY());
    }
    public Pose2D getCurrentPose(){
        return currentPose;
    }

    public double getHeadingVelocity(){
        return Odo.odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
    public boolean distancePathCompletion(Pose2D target){
        return (currentPose.getDistance(target) < target.defaultTolerance);
    }

    public double getXYVelocity() {
        return Math.hypot(getXVelocity(), getYVelocity());
    }

    public double getXVelocity() { return xVelocity; }
    public double getYVelocity() { return yVelocity; }

    public static double filterParameter = 0.8;
    private static final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0);
    private static final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter, 0);
    public double xRobotVelocity , yRobotVelocity , xVelocity , yVelocity , xDeceleration , yDeceleration , forwardGlide , lateralGlide , xGlide , yGlide;
    public void updateGlide(){
        xRobotVelocity = xVelocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }


    public void update() {
        Odo.odo.update();
        heading=Odo.odo.getHeading(RADIANS);
        x=Odo.odo.getPosX(DistanceUnit.INCH);
        y=Odo.odo.getPosY(DistanceUnit.INCH);
        xVelocity = xVelocityFilter.update(Odo.odo.getVelX(DistanceUnit.INCH));
        yVelocity = yVelocityFilter.update(Odo.odo.getVelY(DistanceUnit.INCH));
        updateGlide();
        predictedX = x + xGlide;
        predictedY = y + yGlide;
        if(Robot.opModeType.equals(Robot.OpModeType.AUTO)) currentPose = new Pose2D(predictedX , predictedY , heading);
        else currentPose = new Pose2D(x , y , heading);
    }


}
