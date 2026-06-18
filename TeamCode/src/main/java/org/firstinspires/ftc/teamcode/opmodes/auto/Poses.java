package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    static class ClosePoses{
        public static Pose startingPose = new Pose(108.41515650741351 , 135.45963756177923 , Math.toRadians(270));
        public static Pose shootPose = new Pose(84.45469522240526 , 82.79406919275122, Math.toRadians(270));

        public static Pose ControlPointSecondSpike = new Pose(77.94728171334428 , 54.29159802306425,  0);
        public static Pose CollectSecondSpike = new Pose( 122.85008237232289 , 59.00164744645799);
        public static Pose collectFirstSpike = new Pose(124.54695222405272, 83.2685337726524, 0);
        public static Pose cycleCP = new Pose(110.05930807248762 , 56.37726523887973);
        public static Pose cycle = new Pose(131.18945634266888 , 60.968698517298165 , Math.toRadians(36));

        public static Pose collectSecondSpikes = new Pose(119.80395387149919, 59.20922570016476, 0);
        public static Pose collectSecondSpikesCP = new Pose(88.01317957166393, 57.8846787479407);
    }

    public static class FarPoses{
        public static Pose startingPose = new Pose(99.25118483412321, 8.71800947867298, 0);
        public static Pose HPCollectPose = new Pose(125.5893076902935, 8.71800947867298, 0);
        public static Pose HpCollectPoseCP = new Pose(127.67676284988609, 14.921208530805671);
        public static Pose shootPose = new Pose(88.85663507109003, 11.735781990521298, 0);
        public static Pose shootPoseScan = new Pose(82.3601186579735, 12.220527523936694, Math.toRadians(10));
        public static Pose spikeCollectPose = new Pose(133.1546961325967, 37.060589318600364);
        public static Pose spikeCollectCP = new Pose(83.0865955692395, 39.99228855247814);
        public static Pose cycleCP = new Pose(103.70663257621327, 29.710299122158613);
        public static Pose leave = new Pose(101.40626780013348, 25.104043218301673, Math.toRadians(0));
    }
}