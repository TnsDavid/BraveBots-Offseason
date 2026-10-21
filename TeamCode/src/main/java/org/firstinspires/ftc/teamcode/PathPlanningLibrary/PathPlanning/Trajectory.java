package org.firstinspires.ftc.teamcode.PathPlanningLibrary.PathPlanning;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.math.MathStuff;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
//TODO: DRAWING , TUNERS
public class Trajectory {
    public LinkedList<Pose2D> controlPoints;
    private double constantHeading = 0.0;
    private double startHeading = 0.0;
    private double endHeading = 0.0;
    public double tValue;
    public enum HeadingInterpolation{
        LINEAR, TANGENTIAL , TANGENTIAL_REVERSE , CONSTANT , PIECEWISE;
    }
    HeadingInterpolation headingInterpolator = HeadingInterpolation.CONSTANT;
    public Trajectory(Pose2D... pose2DS){
        controlPoints = new LinkedList<>();
        for (Pose2D pose : pose2DS) {
            controlPoints.addLast(pose);
        }
    }
    public double getClosestT(Pose2D robotPose, double lastT) {
        double bestT = lastT;
        double bestDist = Double.MAX_VALUE;

        Pose2D lastPose = getPose(lastT);
        double distToPath = Math.hypot(
                lastPose.getX() - robotPose.getX(),
                lastPose.getY() - robotPose.getY()
        );
        double from = distToPath > 8.0 ? 0.0 : Math.max(0.0, lastT - 0.10);
        double to   = distToPath > 8.0 ? 1.0 : Math.min(1.0, lastT + 0.30);
        for (double t = from; t <= to; t += 0.01) {
            Pose2D p = getPose(t);
            double d = Math.hypot(p.getX() - robotPose.getX(), p.getY() - robotPose.getY());
            if (d < bestDist) {
                bestDist = d;
                bestT = t;
            }
        }
        return bestT;
    }
    public Pose2D getPose(double t) {
        this.tValue = t;
        List<Pose2D> points = new ArrayList<>(controlPoints);

        while (points.size() > 1) {
            List<Pose2D> next = new ArrayList<>(points.size() - 1);

            for (int i = 0; i < points.size() - 1; i++) {
                Pose2D p0 = points.get(i);
                Pose2D p1 = points.get(i + 1);

                double x = (1.0 - t) * p0.getX() + t * p1.getX();
                double y = (1.0 - t) * p0.getY() + t * p1.getY();

                next.add(new Pose2D(x, y));
            }
            points = next;
        }

        Pose2D position = points.get(0);
        double heading = getHeading(t);
        return new Pose2D(position.getX(), position.getY(), heading);
    }
    public Pose2D evaluate(double t) {
        this.tValue = t;
        return getPose(t);
    }
    public Pose2D tangent(double t) {
        this.tValue = t;
        t = Range.clip(t, 0.0, 1.0);

        int degree = controlPoints.size() - 1;
        if (degree <= 0) return new Pose2D(0, 0, 0);

        List<Pose2D> derivPoints = new ArrayList<>(degree);
        for (int i = 0; i < degree; i++) {
            Pose2D p0 = controlPoints.get(i);
            Pose2D p1 = controlPoints.get(i + 1);

            double dx = degree * (p1.getX() - p0.getX());
            double dy = degree * (p1.getY() - p0.getY());
            derivPoints.add(new Pose2D(dx, dy));
        }
        while (derivPoints.size() > 1) {
            List<Pose2D> next = new ArrayList<>(derivPoints.size() - 1);
            for (int i = 0; i < derivPoints.size() - 1; i++) {
                Pose2D p0 = derivPoints.get(i);
                Pose2D p1 = derivPoints.get(i + 1);
                double x = (1.0 - t) * p0.getX() + t * p1.getX();
                double y = (1.0 - t) * p0.getY() + t * p1.getY();
                next.add(new Pose2D(x, y));
            }
            derivPoints = next;
        }
        return derivPoints.get(0);
    }
    public Trajectory setTangentHeadingInterpolation(boolean reverse){
        this.headingInterpolator = HeadingInterpolation.TANGENTIAL;
        if(reverse) this.headingInterpolator = HeadingInterpolation.TANGENTIAL_REVERSE;
        return this;
    }
    public Trajectory setConstantHeadingInterpolation(double heading){
        this.headingInterpolator = HeadingInterpolation.CONSTANT;
        constantHeading = heading;
        return this;
    }
    public Trajectory setLinearHeadingInterpolation(double startHeading , double endHeading){
        this.headingInterpolator = HeadingInterpolation.LINEAR;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        return this;
    }
    public Trajectory setHeadingInterpolator(HeadingInterpolation headingInterpolator) {
        this.headingInterpolator = headingInterpolator;
        return this;
    }
    public double getHeading(double t) {
        this.tValue = t;
        t = Range.clip(t , 0 , 1 );
        switch(headingInterpolator){
            case TANGENTIAL:
                return getTangentHeading(t , false);
            case TANGENTIAL_REVERSE:
                return getTangentHeading(t , true);
            case CONSTANT:
                return constantHeading;
            case LINEAR:
                return interpolateHeading(startHeading , endHeading , t);
            default:
                return 0.0;
        }
    }
    public double getHeading( double t ,HeadingInterpolation startHeadingType , HeadingInterpolation endHeadingType ){
        if(headingInterpolator.equals(HeadingInterpolation.PIECEWISE)) return getPieceWiseHeading(t, startHeadingType , endHeadingType);
        else return getHeading(t);
    }
    public double getTangentHeading(double t , boolean reverse){
        this.tValue = t;
        t = Range.clip(t , 0 , 1 );

        Pose2D tan = tangent(t);
        double theta =  Math.atan2(tan.getY() , tan.getX());

        if(reverse)  theta = theta + Math.PI;

        return MathStuff.normalizeRadians(theta);
    }
    public double getPieceWiseHeading(double t , HeadingInterpolation startHeadingType , HeadingInterpolation endHeadingType){
        t = Range.clip(t , 0 ,1 );
        if(tValue < t) setHeadingInterpolator(startHeadingType);
        if(tValue > t && tValue <= 1) setHeadingInterpolator(endHeadingType);
        return getHeading(t);
    }
    public double interpolateHeading( double startHeading , double endHeading , double t){
        this.tValue = t;
        return startHeading + t * MathStuff.normalizeRadians(endHeading - startHeading);
    }
    public Pose2D unitTangent(double t) {
        Pose2D tan = tangent(t);
        double len = Math.hypot(tan.getX(), tan.getY());
        if (len < 1e-9) return new Pose2D(0, 0, 0);
        return new Pose2D(tan.getX() / len, tan.getY() / len);
    }
    public double curvature(double t) {
        Pose2D d1 = tangent(t);
        Pose2D d2 = secondDerivative(t);

        double cross = d1.getX() * d2.getY() - d1.getY() * d2.getX();
        double speed = Math.hypot(d1.getX(), d1.getY());

        if (speed < 1e-8) return 0.0;
        return cross / (speed * speed * speed);
    }
    public Pose2D secondDerivative(double t) {
        t = Range.clip(t, 0.0, 1.0);
        int n = controlPoints.size() - 1;
        if (n < 2) return new Pose2D(0, 0);

        // Prima derivată
        List<Pose2D> first = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            Pose2D p0 = controlPoints.get(i);
            Pose2D p1 = controlPoints.get(i + 1);
            first.add(new Pose2D(
                    n * (p1.getX() - p0.getX()),
                    n * (p1.getY() - p0.getY())
            ));
        }
        List<Pose2D> second = new ArrayList<>(n - 1);
        for (int i = 0; i < n - 1; i++) {
            Pose2D p0 = first.get(i);
            Pose2D p1 = first.get(i + 1);
            second.add(new Pose2D(
                    (n - 1) * (p1.getX() - p0.getX()),
                    (n - 1) * (p1.getY() - p0.getY())
            ));
        }
        while (second.size() > 1) {
            List<Pose2D> next = new ArrayList<>(second.size() - 1);
            for (int i = 0; i < second.size() - 1; i++) {
                Pose2D p0 = second.get(i);
                Pose2D p1 = second.get(i + 1);
                next.add(new Pose2D(
                        (1 - t) * p0.getX() + t * p1.getX(),
                        (1 - t) * p0.getY() + t * p1.getY()
                ));
            }
            second = next;
        }
        return second.get(0);
    }
    public double approximateLength(int samples) {
        if (samples < 2) samples = 30;
        double length = 0;
        Pose2D prev = getPose(0);

        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            Pose2D curr = getPose(t);
            length += Math.hypot(curr.getX() - prev.getX(), curr.getY() - prev.getY());
            prev = curr;
        }
        return length;
    }
}
