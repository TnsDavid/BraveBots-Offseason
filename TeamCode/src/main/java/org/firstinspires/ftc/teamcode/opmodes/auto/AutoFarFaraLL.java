package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;

public class AutoFarFaraLL extends Auto{
    public class HPCollect implements Task {
        Command s;

        public HPCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .lineToConstantAsync(Poses.FarPoses.HPCollectPose, Poses.FarPoses.HPCollectPose.getHeading())
                    .waitSeconds(0.3)
                    .addTask(Intake::stop)
                    .lineToConstantSync(Poses.FarPoses.shootPose, Poses.FarPoses.shootPose.getHeading())
                    .addTask(() -> follower.getCurrentTValue() > 0.8)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(new Shoot())
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
                    .curveToTangentialAsync(Poses.FarPoses.spikeCollectCP, Poses.FarPoses.spikeCollectPose)
                    .waitSeconds(0.3)
                    .addTask(Intake::stop)
                    .lineToConstantSync(Poses.FarPoses.shootPose, Poses.FarPoses.shootPose.getHeading())
                    .addTask(() -> follower.getCurrentTValue() > 0.8)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(new Shoot())
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

        public CycleCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .curveToConstantAsync(Poses.FarPoses.cycleCP, Poses.FarPoses.spikeCollectPose, Math.toRadians(0))
                    .waitSeconds(0.2)
                    .addTask(Intake::stop)
                    .lineToConstantSync(Poses.FarPoses.shootPose, Poses.FarPoses.shootPose.getHeading())
                    .addTask(() -> follower.getCurrentTValue() > 0.8)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(new Shoot())
                    .waitSeconds(0.1);
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    @Override
    public void buildAuto(){
        follower.setStartingPose(Poses.FarPoses.startingPose);
        auto.addTask(new Shoot())
                .addTask(new HPCollect())
                .addTask(new SpikeCollect())
                .addTask(new HPCollect())
                .addTask(new CycleCollect())
                .addTask(new HPCollect())
                .addTask(new CycleCollect())
                .waitSeconds(0.1)
                .lineToTangentialASync(Poses.FarPoses.leave);

    }
    @Override
    public void setAlliance(){
        alliance = Alliance.RED;
    }
}