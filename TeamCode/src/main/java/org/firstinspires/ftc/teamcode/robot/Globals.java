package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Globals {
    public static Pose lastPose = new Pose();

    public enum HEADING {
        OPPOSITE, NEUTRAL_PLUS, TOWARD, NEUTRAL_MINUS;
    }
    public static HEADING headingState;
    public static void classifyHeading() {
        double heading = Odo.odo.getHeading(AngleUnit.DEGREES);
        if( -45 <= heading && heading < 45 )
            headingState = Robot.alliance == Alliance.RED ? HEADING.TOWARD : HEADING.OPPOSITE;
        else if( 45 <= heading && heading < 135 )
            headingState = HEADING.NEUTRAL_PLUS;
        else if( 135 <= heading || heading < -135)
            headingState = Robot.alliance == Alliance.RED ? HEADING.OPPOSITE : HEADING.TOWARD;
        else if( -135 <= heading && heading < -45 )
            headingState = HEADING.NEUTRAL_MINUS;
    }

    public enum DISTANCE {
        FAR, MIDDLE, CLOSE;
    }
    public static DISTANCE distanceState;
    public static void classifyDistance() {
        double xCoordinate = Odo.odo.getPosX(DistanceUnit.METER);
        if( xCoordinate < 1.5 )
            distanceState = Robot.alliance == Alliance.RED ? DISTANCE.FAR : DISTANCE.CLOSE;
        else if( 1.5 <= xCoordinate && xCoordinate < 2.1 )
            distanceState = DISTANCE.MIDDLE;
        else
            distanceState = Robot.alliance == Alliance.RED ? DISTANCE.CLOSE : DISTANCE.FAR;
    }

    public static double calculateTurretOffset() {
        double offset = -6.7;
        classifyHeading();
        classifyDistance();
        switch (distanceState) {
            case FAR:
                switch (headingState) {
                    case OPPOSITE:
                        offset = 0.5;
                        break;
                    case NEUTRAL_PLUS:
                        offset = 2;
                        break;
                    case TOWARD:
                        offset = -3;
                        break;
                    case NEUTRAL_MINUS:
                        offset = -2;
                        break;
                }
            case MIDDLE:
                switch (headingState) {
                    case OPPOSITE:
                        offset = 2.5;
                        break;
                    case NEUTRAL_PLUS:
                        offset = 2;

                        break;
                    case TOWARD:
                        offset = -2;
                        break;
                    case NEUTRAL_MINUS:
                        offset = 0;
                        break;
                }
            case CLOSE:
                switch (headingState) {
                    case OPPOSITE:
                        offset = 2.5;
                        break;
                    case NEUTRAL_PLUS:
                        offset = 0;
                        break;
                    case TOWARD:
                        offset = -2.5;
                        break;
                    case NEUTRAL_MINUS:
                        offset = 0.5;
                        break;
                }
        }
        return offset;
    }
}
