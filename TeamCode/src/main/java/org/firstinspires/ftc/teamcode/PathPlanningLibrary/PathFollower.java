package org.firstinspires.ftc.teamcode.PathPlanningLibrary;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.XYVelocityConstraint;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.dforward;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.dheading;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.dstrafe;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.fforward;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.fheading;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.forward;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.fstrafe;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.heading;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.headingConstraint;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.headingVelocityConstraint;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.pforward;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.pheading;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.pstrafe;
import static org.firstinspires.ftc.teamcode.PathPlanningLibrary.Constants.strafe;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PathPlanningLibrary.PathPlanning.Pose2D;
import org.firstinspires.ftc.teamcode.PathPlanningLibrary.PathPlanning.Trajectory;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.util.math.MathStuff;
import org.firstinspires.ftc.teamcode.util.math.Vector;

public class PathFollower {

    public Localizer localizer;
    public Trajectory trajectory;
    public double currentTvalue = 0;
    public ElapsedTime pathTimer;


    public double targetX , targetY , targetHeading;
    Pose2D currentPose;

    public boolean usePredictiveBraking = false;
    public PathFollower(Localizer localizer){
        this.localizer = localizer;
    }

    public Pose2D getCurrentPose(){
        return currentPose;
    }

    public PathFollower initialize(){
        setControllers();
        currentTvalue = 0;
        trajectory = null;
        pathTimer = new ElapsedTime();
        usePredictiveBraking = false;
        return this;
    }

    public Pose2D getCurrentFollowedPose(){
        return this.trajectory.getPose(currentTvalue);
    }
    public void follow(Pose2D target , Trajectory.HeadingInterpolation interpolation){
        pathTimer = new ElapsedTime();
        this.trajectory = new Trajectory(localizer.getCurrentPose() , target)
                .setHeadingInterpolator(interpolation);
        currentTvalue = 0;
    }
    public void follow(Pose2D controlpoint , Pose2D target){
        pathTimer = new ElapsedTime();
        this.trajectory = new Trajectory(localizer.getCurrentPose() ,controlpoint ,target);
        currentTvalue = 0;
    }
    public double getHeadingCorrection(double target , double current){
        this.targetHeading = target;
        double Headingerror = MathStuff.normalizeRadians(target- current);
        if (Math.abs(Headingerror) >= Math.PI / 2) {
            Headingerror = Math.signum(-Headingerror) * (2 * Math.PI - Math.abs(Headingerror));
        }
        return heading.calculate(Headingerror , 0);
    }

    public double getCorrectivePower() {
        Pose2D pathPose = trajectory.getPose(currentTvalue);
        Pose2D robotPose = getCurrentPose();

        double dx = robotPose.getX() - pathPose.getX();
        double dy = robotPose.getY() - pathPose.getY();

        Vector unitTangent = trajectory.unitTangent(currentTvalue).getAsVector();

        // Signed cross-track error (formula standard)
        double crosstrack = dx * unitTangent.getYComponent() - dy * unitTangent.getXComponent();

        double correctivePower = strafe.calculate(crosstrack, 0);
        return Range.clip(correctivePower, -1.0, 1.0);
    }

    public double getPathingPower(Vector target){
        double remainingX = getXPathingPower(target.xComponent);
        double remainingY = getYPathingPower(target.yComponent);
        return Range.clip(Math.abs(Math.hypot(remainingX , remainingY)), -1, 1);
    }
    public double getXPathingPower(double target){
        double power  = forward.calculate(target , 0);
            return Range.clip(power, -1, 1);
    }
    public double getYPathingPower(double target){
        double power  = strafe.calculate(target ,0);
            return Range.clip(power, 0.0, 1);
    }

    public Pose2D getTargetPose(){
        return trajectory.getPose(1);
    }
    public void turnTo(double heading){
        pathTimer = new ElapsedTime();
        trajectory = new Trajectory(getCurrentPose() , new Pose2D(getCurrentPose().getX() , getCurrentPose().getY() , heading)).setConstantHeadingInterpolation(heading);
    }

    public boolean isBusy() {
        return trajectory != null
                && currentTvalue < 0.98
                && !finished();
    }
    public double getRadiansFromHeadingGoal(){
        return Math.abs(MathStuff.normalizeRadians(localizer.currentPose.getHeading() - getTargetPose().getHeading()));
    }
    public boolean finished() {
        return getCurrentPose().isReached(getTargetPose())
                && getRadiansFromHeadingGoal() < headingConstraint
                && localizer.getHeadingVelocity() < headingVelocityConstraint
                && localizer.getXYVelocity() < XYVelocityConstraint;
    }

    public boolean finished(double sec){
            return  finished()
                    || pathTimer.seconds()> sec;
    }
    public double getCurrentTvalue(){
        return currentTvalue;
    }
    public void goTo(double targetx , double targety , double targeth , double speed){
        localizer.update();
        pathTimer = new ElapsedTime();
        Vector driveVector = new Vector(targetx - getCurrentPose().getX(), targety - getCurrentPose().getY());
        Vector rotateVector = driveVector.rotateBy(localizer.getHeading(AngleUnit.RADIANS));
        double inputTurn = getHeadingCorrection(targeth , localizer.getHeading(AngleUnit.RADIANS));
        double drivePower = getXPathingPower(rotateVector.xComponent);
        double strafecorrection = getYPathingPower(rotateVector.yComponent);

        double fl = (drivePower + strafecorrection + inputTurn) * speed;
        double bl = (drivePower - strafecorrection + inputTurn) * speed;
        double fr = (drivePower - strafecorrection - inputTurn) * speed;
        double br = (drivePower + strafecorrection - inputTurn) * speed;

        Drivetrain.leftFront.setPower(fl);
        Drivetrain.leftBack.setPower(bl);
        Drivetrain.rightBack.setPower(br);
        Drivetrain.rightFront.setPower(fr);
    }
    public void goTo(Pose2D target , double speed ){
        localizer.update();
        pathTimer = new ElapsedTime();
        Vector driveVector = new Vector(target.x - getCurrentPose().getX(), target.y - getCurrentPose().getY());
        Vector rotateVector = driveVector.rotateBy(localizer.getHeading(AngleUnit.RADIANS));
        double inputTurn = getHeadingCorrection(target.heading, localizer.getHeading(AngleUnit.RADIANS));
        double drivePower = getXPathingPower(rotateVector.xComponent);
        double strafecorrection = getYPathingPower(rotateVector.yComponent);

        double fl = (drivePower + strafecorrection + inputTurn) * speed;
        double bl = (drivePower - strafecorrection + inputTurn) * speed;
        double fr = (drivePower - strafecorrection - inputTurn) * speed;
        double br = (drivePower + strafecorrection - inputTurn) * speed;

        Drivetrain.leftFront.setPower(fl);
        Drivetrain.leftBack.setPower(bl);
        Drivetrain.rightBack.setPower(br);
        Drivetrain.rightFront.setPower(fr);
    }

    public void goTo(Pose2D target){
        goTo(target , 0.7);
    }

    public void setControllers(){
        strafe.setPIDF(pstrafe, 0, dstrafe,fstrafe);
        forward.setPIDF(pforward, 0, dforward, fforward);
        heading.setPIDF(pheading, 0, dheading, fheading);
    }
    public void update(){
        localizer.update();
        currentPose = localizer.currentPose;
        if(isBusy()){
            currentTvalue = trajectory.getClosestT(currentPose , currentTvalue);
            targetX = getCurrentFollowedPose().x;
            targetY = getCurrentFollowedPose().y;
            targetHeading = trajectory.getHeading(currentTvalue);
            if (Double.isNaN(Odo.odo.getPosX(INCH)) || Double.isNaN(Odo.odo.getPosY(INCH)) || Double.isNaN(Odo.odo.getHeading(AngleUnit.RADIANS))) {
                throw new RuntimeException("Pe odometrii le doare rau in pula");
            }
            Vector driveVector = new Vector(
                    targetX - getCurrentPose().getX(),
                    targetY - getCurrentPose().getY()
            );

            Vector rotateVector = driveVector.rotateBy(localizer.getHeading(AngleUnit.RADIANS));
            double x = getXPathingPower(rotateVector.xComponent);
            double y = getCorrectivePower();
            double rotation = getHeadingCorrection(targetHeading, localizer.getHeading(AngleUnit.RADIANS));
            Drivetrain.betterDrive(x , y , rotation);
        } else Drivetrain.betterDrive(0, 0, 0);
    }


}
