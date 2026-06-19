package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    static class ClosePoses {
        public static Pose startingPose = new Pose(108.41515650741351 , 135.45963756177923 , Math.toRadians(270));
        public static Pose startingPose2 = new Pose(125.97034596375615 , 118.3789126853377 , Math.toRadians(216));
        public static Pose shootPose = new Pose(84.45469522240526 , 82.79406919275122  );
        public static Pose collectSecondSpike = new Pose(122.7858319604613, 59.20922570016476, 0);
        public static Pose collectSecondSpikeCP = new Pose(83.26853377265239, 64.05271828665568);
        public static Pose gateCollect = new Pose(131.64250411861616, 59.4, Math.toRadians(35));
        public static Pose gateCollectCP = new Pose(111.07734761120264 , 59.642833607907754);
        public static Pose breakPoint = new Pose(100.04283360790775, 83.2504118616145, 0);
        public static Pose collectFirstSpike = new Pose(122.93574958813838, 83.2504118616145, 0);
        public static Pose parkingSpotdeHandicapati = new Pose(127.244, 80.37, Math.toRadians(0));
    }

    static class FarPoses {
        public static Pose startingPose = new Pose(99.25118483412321, 8.71800947867298, 0);
        public static Pose HPCollectRewindPose = new Pose(120.42904423085072, 8.71800947867298, 0);
        public static Pose HPCollectPose = new Pose(129.7528436018957, 8.71800947867298, 0);
        public static Pose shootPose = new Pose(88.85663507109003, 11.06516587677723, 0);
        public static Pose shootPoseScan = new Pose(88.35367298578198, 12.74170616113743, Math.toRadians(10));
        public static Pose spikeCollectPose = new Pose(133.99264903868246, 35.254721467066055);
        public static Pose spikeCollectCP = new Pose(85.95307807384359, 36.907574003675194);
        public static Pose cycleCP = new Pose(103.70663257621327, 29.710299122158613);
        public static Pose leave = new Pose(101.40626780013348, 25.104043218301673);
    }
}