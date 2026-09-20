package org.firstinspires.ftc.teamcode.madoarerauinpula.tests;

import org.firstinspires.ftc.teamcode.madoarerauinpula.PathPlanning.Pose2D;
import org.firstinspires.ftc.teamcode.madoarerauinpula.PathPlanning.Trajectory;

public class Trajectorytest {
    public static void main(String[] args) {

        Trajectory traj = new Trajectory(
                new Pose2D(0, 0),
                new Pose2D(20, 40),
                new Pose2D(60, 40),
                new Pose2D(80, 0)
        );

        System.out.println("========== TEST TRAJECTORY ==========\n");

        Pose2D start = traj.getPose(0.0);
        Pose2D end   = traj.getPose(1.0);

        System.out.println("Start (t=0): " + start);
        System.out.println("End   (t=1): " + end);
        System.out.println();

        System.out.println("----- Poziții pe path -----");
        double[] ts = {0.0, 0.25, 0.5, 0.75, 1.0};

        for (double t : ts) {
            Pose2D pose = traj.getPose(t);
            Pose2D tan  = traj.tangent(t);
            double headingDeg = Math.toDegrees(traj.getHeading(t));
            double curv = traj.curvature(t);

            System.out.printf(
                    "t = %.2f | Pos: (%6.1f, %6.1f) | Heading: %6.1f° | Tangent: (%6.1f, %6.1f) | Curv: %.4f%n",
                    t,
                    pose.getX(), pose.getY(),
                    headingDeg,
                    tan.getX(), tan.getY(),
                    curv
            );
        }

        System.out.println();

        double length = traj.approximateLength(50);
        System.out.printf("Lungime aproximativă: %.1f inch%n", length);

        System.out.println("\n========== SFÂRȘIT TEST ==========");
    }
}
