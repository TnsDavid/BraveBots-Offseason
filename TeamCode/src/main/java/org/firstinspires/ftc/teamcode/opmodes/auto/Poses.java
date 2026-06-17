package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    static class ClosePoses{
        public static Pose startingPose = new Pose(123.36079077429979 , 121.70016474464583 , Math.toRadians(226));
        public static Pose shootPose0 = new Pose(89.67380560131795 , 87.3014827018122 , 0);
        public static Pose shootPose = new Pose(79.47281713344316 , 78.76112026359144);
        public static Pose ControlPointSecondSpike = new Pose(77.94728171334428 , 54.29159802306425,  0);
        public static Pose CollectSecondSpike = new Pose( 122.85008237232289 , 59.00164744645799);
        public static Pose collectFirstSpike = new Pose(124.54695222405272, 83.2685337726524, 0);
        public static Pose cycleCP = new Pose(110.05930807248762 , 56.37726523887973);
        public static Pose cycle = new Pose(131.18945634266888 , 60.968698517298165 , Math.toRadians(36));
    }

    static class FarPoses{
        public static Pose testFarStartingPose = new Pose(0, 0, 0);
    }
}
