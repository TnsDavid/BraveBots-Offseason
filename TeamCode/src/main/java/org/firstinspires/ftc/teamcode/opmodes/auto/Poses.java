package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    static class ClosePoses{
        public static Pose startingPose = new Pose(123.13673805601317 , 122.10873146622737 , Math.toRadians(270));
        public static Pose shootPose = new Pose(83.75617792421748 , 83.6836902800659 , 0);

        public static Pose ControlPointSecondSpike = new Pose(104.6194398682043 , 59.30807248764417,  0);
        public static Pose CollectSecondSpike = new Pose( 128.59308072487644 , 59.331136738056);
        public static Pose collectFirstSpike = new Pose(124.54695222405272, 83.2685337726524, 0);
    }

    static class FarPoses{
        public static Pose testFarStartingPose = new Pose(0, 0, 0);
    }
}
