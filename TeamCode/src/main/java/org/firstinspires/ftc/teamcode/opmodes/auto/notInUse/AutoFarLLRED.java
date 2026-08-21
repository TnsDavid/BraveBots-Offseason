package org.firstinspires.ftc.teamcode.opmodes.auto.notInUse;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses.AutoFar;
import org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses.Poses;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

public class AutoFarLLRED extends AutoFar {
    public class HPCollect implements Task {
        Command s;

        ElapsedTime SmartCollectTimer;

        public HPCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .lineToConstantSync(Poses.FarPoses.HPCollectPose, Poses.FarPoses.HPCollectPose.getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.75)
                    .addTask(() -> follower.setMaxPower(0.7))
                    .addTask(() -> !follower.isBusy())
                    .waitSeconds(0.25)
                    .addTask(()->follower.setMaxPower(0.6))
                    .lineToConstantSync(Poses.FarPoses.shootPose, Poses.FarPoses.shootPose.getHeading())
                    .addTask(() -> follower.getCurrentTValue()>0.3)
                    .addTask(() -> follower.setMaxPower(1))
                    .addTask(()->follower.getCurrentTValue() > 0.6)
                    .addTask(Intake::stop)
                    .addTask(() -> !follower.isBusy())
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
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .curveToConstantSync(Poses.FarPoses.spikeCollectCP, Poses.FarPoses.spikeCollectPose,Math.toRadians(0))
                    .addTask(() -> follower.getCurrentTValue() > 0.7)
                    .addTask(() -> follower.setMaxPower(0.8))
                    .addTask(() -> !follower.isBusy())
                    .addTask(() -> follower.setMaxPower(1))
                    .lineToTangentialToConstantAsync(Poses.FarPoses.shootPoseScan, 0.7, true)
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
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(() -> LLMath.scan(follower))
                    .addTask(() -> SmartCollectTimer = new ElapsedTime())
                    .addTask(() -> follower.getCurrentTValue() > 0.5)
                    .addTask(() -> !follower.isBusy() || Odo.odo.getVelX(DistanceUnit.METER) < 0.5)
                    .addTask(() -> follower.turn(Math.toRadians(20), true))
                    .waitSeconds(0.05)
                    .addTask(() -> follower.turn(Math.toRadians(20), false))
                    .waitSeconds(0.05)
                    .addTask(()->follower.setMaxPower(1))
                    .lineToTangentialToConstantAsync(Poses.FarPoses.shootPoseScan, 0.7, true)
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
                    .lineToConstantSync(Poses.FarPoses.leave, Poses.FarPoses.leave.getHeading());
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
        offset = 1;
        velocity = Shooter.velocityFAR - 20;
        pos = Shooter.hoodFAR;
        auto
                //.addTask(() -> Turret.setNeutralPosition(0.1895))
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
        alliance = Alliance.RED;
    }
}