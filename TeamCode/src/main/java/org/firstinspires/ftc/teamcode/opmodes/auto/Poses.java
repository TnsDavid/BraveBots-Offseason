package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    static class ClosePoses{
        public static Pose testStartingPose = new Pose(72, 72 );
        public static Pose bottomRight = new Pose(92, 72);
        public static Pose topRight = new Pose(92, 92);
        public static Pose topLeft = new Pose(72, 92);
        public static Pose pozitieNoua = new Pose(120, 72);
    }

    static class FarPoses{
        public static Pose testFarStartingPose = new Pose(0, 0, 0);
    }
}
