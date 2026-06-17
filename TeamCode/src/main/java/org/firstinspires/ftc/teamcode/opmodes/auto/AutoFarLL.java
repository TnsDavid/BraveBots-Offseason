package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

@Autonomous
public class AutoFarLL extends AutoFar{
    public class HPCollect implements Task {
        Command s;

        public HPCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .lineToConstantAsync(Poses.FarPoses.HPCollectPose, Poses.FarPoses.HPCollectPose.getHeading())
                    .waitSeconds(0.25)
                    .lineToConstantAsync(Poses.FarPoses.HPCollectRewindPose, Poses.FarPoses.HPCollectRewindPose.getHeading())
                    .lineToConstantAsync(Poses.FarPoses.HPCollectPose, Poses.FarPoses.HPCollectPose.getHeading())
                    .waitSeconds(0.3)
                    .addTask(Intake::stop)
                    .lineToConstantSync(Poses.FarPoses.shootPose, Poses.FarPoses.shootPose.getHeading())
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
                    .curveToConstantAsync(Poses.FarPoses.spikeCollectCP, Poses.FarPoses.spikeCollectPose,Math.toRadians(0))
                    .waitSeconds(0.1)
                    .addTask(Intake::stop)
                    .lineToConstantSync(Poses.FarPoses.shootPoseScan, Poses.FarPoses.shootPoseScan.getHeading())
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

    public class CycleCollect implements Task {
        Command s;

        public CycleCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .addTask(() -> LLMath.scan(follower))
                    .addTask(() -> !follower.isBusy())
                    .waitSeconds(0.3)
                    .addTask(Intake::stop)
                    .lineToConstantSync(Poses.FarPoses.shootPoseScan, Poses.FarPoses.shootPose.getHeading())
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

    @Override
    public void buildAuto(){
        follower.setStartingPose(Poses.FarPoses.startingPose);
        auto.addTask(new ShootFar())
                .addTask(new HPCollect())
                .addTask(new SpikeCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect())
                .addTask(new CycleCollect());
    }

    @Override
    public void setAlliance(){
        alliance = Alliance.RED;
    }
}
