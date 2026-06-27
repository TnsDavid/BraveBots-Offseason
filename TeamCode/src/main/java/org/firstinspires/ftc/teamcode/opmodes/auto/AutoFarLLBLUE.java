package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

@Autonomous
public class AutoFarLLBLUE extends AutoFar{
    double fieldLength = 140;

    public class HPCollect implements Task {
        Command s;

        ElapsedTime SmartCollectTimer;

        public HPCollect() {
            s = new Command(follower)
                    .addTask(()-> Turret.setNeutralPosition(0.821))
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .curveToConstantSync(Poses.FarPoses.HpCollectPoseCP.mirror(fieldLength), Poses.FarPoses.HPCollectPose.mirror(fieldLength), Poses.FarPoses.HPCollectPose.mirror(fieldLength).getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.75)
                    .addTask(()->SmartCollectTimer = new ElapsedTime())
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(() -> !follower.isBusy() || SmartCollectTimer.seconds() > 1)
                    .waitSeconds(0.4)
                    .addTask(()->follower.setMaxPower(1))
                    .addTask(() -> !follower.isBusy())
                    .lineToConstantAsync(Poses.FarPoses.shootPose.mirror(fieldLength), Poses.FarPoses.shootPose.mirror(fieldLength).getHeading())
                    .waitSeconds(0.1)
                    .addTask(new ShootFar());
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    public class SpikeCollect implements Task {
        Command s;

        public SpikeCollect() {
            s = new Command(follower)
                    .addTask(()-> Turret.setNeutralPosition(0.821))
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .curveToConstantSync(Poses.FarPoses.spikeCollectCP.mirror(fieldLength), Poses.FarPoses.spikeCollectPose.mirror(fieldLength),Math.toRadians(180))
                    .addTask(() -> follower.getCurrentTValue() > 0.7)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(() -> !follower.isBusy())
                    .addTask(() -> follower.setMaxPower(1))
                    .lineToConstantAsync(Poses.FarPoses.shootPoseScan.mirror(fieldLength), Poses.FarPoses.shootPoseScan.mirror(fieldLength).getHeading())
                    .waitSeconds(0.1)
                    .addTask(new ShootFar());
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    public class CycleCollect implements Task {
        Command s;
        ElapsedTime SmartCollectTimer;
        public CycleCollect() {
            s = new Command(follower)
                    //.addTask(()-> Turret.setNeutralPosition(0.821))
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(() -> LLMath.scan(follower))
                    .addTask(() -> SmartCollectTimer = new ElapsedTime())
                    .addTask(() -> follower.setMaxPower(0.7))
                    .addTask(() ->(!follower.isBusy()||SmartCollectTimer.seconds()>1.3))
                    .waitSeconds(0.25)
                    .addTask(()->follower.setMaxPower(1))
                    .lineToConstantAsync(Poses.FarPoses.shootPoseScan.mirror(fieldLength), Poses.FarPoses.shootPose.mirror(fieldLength).getHeading())
                    .waitSeconds(0.1)
                    .addTask(new ShootFar());
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }
    public class Leave implements Task{
        Command s;

        public Leave() {
            s = new Command(follower)
                    .addTask(() -> follower.setMaxPower(1))
                    .addTask(Intake::stop)
                    .addTask(Storage::stop)
                    .lineToConstantSync(Poses.FarPoses.leave.mirror(fieldLength), Poses.FarPoses.leave.mirror(fieldLength).getHeading());
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    @Override
    public void buildAuto(){
        follower.setStartingPose(Poses.FarPoses.startingPose.mirror(fieldLength));
        auto
                //.addTask(() -> Turret.setNeutralPosition(0.81))
                .addTask(new ShootFar())
                .addTask(new HPCollect())
                .addTask(new SpikeCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect())
                .addTask(new Leave());
    }

    @Override
    public void setAlliance(){
        alliance = Alliance.BLUE;
    }
}