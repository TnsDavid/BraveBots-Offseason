package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    public static class ClosePoses {
        public static Pose startingPose = new Pose(107.46622734761114 , 135.22240527182862 , Math.toRadians(270));
        public static Pose shootPose = new Pose(84.45469522240526 , 82.79406919275122   );
        public static Pose collectSecondSpike = new Pose(124.7858319604613, 59.20922570016476, 0);
        public static Pose collectSecondSpikeCP = new Pose(83.26853377265239, 64.05271828665568);
        public static Pose gateCollect = new Pose(130.84056783759362 , 59.15440638784079, Math.toRadians(31));
        public static Pose gateCollect2 = new Pose(131.78949699739593, 59.86610325769252, Math.toRadians(31));
        public static Pose gateCollectCP = new Pose(105.56836902800659, 67.44678747940692, 64.6);
        public static Pose collectFirstSpike = new Pose(122.93574958813838, 83.2504118616145, 0);
        public static Pose parkingSpotdeHandicapati = new Pose(127.244, 80.37, Math.toRadians(0));
        public static Pose shootPoseLeave = new Pose(86.35255354200987, 104.61943986820428, Math.toRadians(310));
    }

    public static class FarPoses {
        public static Pose startingPose = new Pose(99.25118483412321, 8.71800947867298, 0);
        public static Pose HPCollectPose = new Pose(133.6552055486131, 8.955241768623553, 0);
        public static Pose HpCollectPoseCP = new Pose(135.74266070820568, 18.95415745996547);
        public static Pose shootPose = new Pose(88.85663507109003, 11.735781990521298, 0);
        public static Pose shootPoseScan = new Pose(82.3601186579735, 12.220527523936694, Math.toRadians(10));
        public static Pose spikeCollectPose = new Pose(133.1546961325967, 37.060589318600364);
        public static Pose spikeCollectCP = new Pose(83.0865955692395, 39.99228855247814);
        public static Pose cycleCP = new Pose(103.70663257621327, 29.710299122158613);
        public static Pose leave = new Pose(101.40626780013348, 25.104043218301673, Math.toRadians(0));
    }
}