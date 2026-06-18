package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

@Autonomous
public class AutoFarLLBLUE extends AutoFar{
    public class HPCollect implements Task {
        Command s;

        public HPCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .lineToConstantSync(Poses.FarPoses.HPCollectPose.mirror(), Poses.FarPoses.HPCollectPose.mirror().getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.65)
                    .addTask(()->follower.setMaxPower(0.7))
                    .addTask(() -> !follower.isBusy())
                    .waitSeconds(0.5)//cat da intake
                    .addTask(()->follower.setMaxPower(1))
                    .lineToConstantSync(Poses.FarPoses.shootPose.mirror(), Poses.FarPoses.shootPose.mirror().getHeading())
                    .addTask(() -> follower.getCurrentTValue() > 0.8)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(new ShootFar())
                    .waitSeconds(0.1);
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
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .curveToConstantSync(Poses.FarPoses.spikeCollectCP.mirror(), Poses.FarPoses.spikeCollectPose.mirror(),Math.toRadians(180))
                    .addTask(() -> follower.getCurrentTValue() > 0.7)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(() -> !follower.isBusy())
                    .waitSeconds(0.2)//cat da intake maybe
                    .addTask(() -> follower.setMaxPower(1))
                    .lineToConstantSync(Poses.FarPoses.shootPoseScan.mirror(), Poses.FarPoses.shootPoseScan.mirror().getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.8)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(new ShootFar())
                    .waitSeconds(0.1);
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
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(() ->{
                        LLMath.scan(follower);
                        SmartCollectTimer = new ElapsedTime();
                    })
                    .addTask(() -> follower.setMaxPower(0.7))
                    .addTask(() ->(!follower.isBusy()||SmartCollectTimer.seconds()>1.5))
                    .waitSeconds(0.3)//cat da intake
                    .addTask(()->follower.setMaxPower(1))
                    .lineToConstantSync(Poses.FarPoses.shootPoseScan.mirror(), Poses.FarPoses.shootPose.mirror().getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.8)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(new ShootFar())
                    .addTask(()->!follower.isBusy())
            ;
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    @Override
    public void buildAuto(){
        follower.setStartingPose(Poses.FarPoses.startingPose.mirror());
        auto.addTask(new ShootFar())
                .addTask(new HPCollect())
                .addTask(new SpikeCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect());
    }

    @Override
    public void setAlliance(){
        alliance = Alliance.BLUE;
    }
}
