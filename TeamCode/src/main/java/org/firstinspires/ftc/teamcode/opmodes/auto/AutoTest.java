package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.robot.Alliance;

public class AutoTest extends Auto{
    @Override
    public void buildAuto(){
        velocity= 950;
        pos= 0.14;
        follower.setStartingPose(Poses.ClosePoses.testStartingPose.mirror());
    }
    @Override
    public void setAlliance(){
        alliance = Alliance.RED;
    }
}
