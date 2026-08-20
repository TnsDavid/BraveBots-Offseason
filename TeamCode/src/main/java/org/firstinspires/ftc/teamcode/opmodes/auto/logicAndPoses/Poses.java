package org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses;

import com.pedropathing.geometry.Pose;

public class Poses {
    public static class ClosePoses {
        public static Pose startingPose = new Pose(110.35065897858323 , 134.28927512355846 , Math.toRadians(270));
        public static Pose shootPose = new Pose(84.45469522240526 , 82.79406919275122);
        public static Pose collectSecondSpike = new Pose(127.3858319604613, 58.38922570016476, 0);
        public static Pose collectSecondSpikeCP = new Pose(88.74, 54.21);
        public static Pose gateCollect = new Pose(131.7 , 58.15, Math.toRadians(29.5));
        public static Pose PULAAAAA = gateCollect.mirror(144);
        public static Pose gateCollectBlue = new Pose(PULAAAAA.getX(), PULAAAAA.getY(), Math.toRadians(151.5));
        public static Pose collectThirdSpike = new Pose(129.3, 35.11037891268535, Math.toRadians(0));
        public static Pose collectThirdSpikeCP = new Pose(86.58978583196047, 28.467874794069186);
        public static Pose gateCollect2 = new Pose(131.78949699739593, 59.86610325769252, Math.toRadians(16));
        public static Pose gateCollectCP = new Pose(105.56836902800659, 67.44678747940692, 0);
        public static Pose collectFirstSpike = new Pose(122.93574958813838, 83.2504118616145, 0);
        public static Pose parkingSpotdeHandicapati = new Pose(127.244, 83.37, Math.toRadians(0));
        public static Pose shootPoseLeave = new Pose(78.998352553542, 103.19604612850083, Math.toRadians(310));
        public static Pose insideLeave = new Pose(83.58, 105.84);
    }

    public static class FarPoses {
        public static Pose startingPose = new Pose(99.25118483412321, 8.71800947867298, 0);
        public static Pose HPCollectPose = new Pose(130.25, 11.4, 0);
        public static Pose shootPose = new Pose(92.65235171029924, 11.97301428047187, 0);
        public static Pose shootPoseScan = new Pose(82.3601186579735, 16.220527523936694, Math.toRadians(12));
        public static Pose spikeCollectPose = new Pose(133.1546961325967, 37.060589318600364);
        public static Pose spikeCollectCP = new Pose(83.0865955692395, 39.99228855247814);
        public static Pose cycleCP = new Pose(103.70663257621327, 29.710299122158613);
        public static Pose leave = new Pose(101.40626780013348, 25.104043218301673, Math.toRadians(0));
    }
}