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
}