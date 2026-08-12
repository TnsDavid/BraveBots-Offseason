package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Odo;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Spit;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

public class AutoFarLLBLUE extends AutoFar{
    public double fieldLength = 144;

    public double amps = 10;
    public class HPCollect implements Task {
        Command s;

        public HPCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .lineToConstantSync(Poses.FarPoses.HPCollectPose.mirror(fieldLength), Poses.FarPoses.HPCollectPose.mirror(fieldLength).getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.75)
                    .addTask(() -> follower.setMaxPower(0.7))
                    .addTask(() -> !follower.isBusy())
                    .addTask(() -> follower.turn(Math.toRadians(13), false))
                    .waitSeconds(0.05)
                    .addTask(() -> follower.turn(Math.toRadians(13), true))
                    .waitSeconds(0.05)
                    .addTask(()->follower.setMaxPower(0.6))
                    .lineToConstantSync(Poses.FarPoses.shootPose.mirror(fieldLength), Poses.FarPoses.shootPose.mirror(fieldLength).getHeading())
                    .addTask(() -> follower.getCurrentTValue()>0.3)
                    .addTask(() -> follower.setMaxPower(1))
                    .addTask(() -> !follower.isBusy())
                    .addTask(()->{
                        if(Intake.motor.getCurrent(CurrentUnit.AMPS) > amps) s.addTask(Spit::new);
                    })
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
                    .addTask(() -> offset = 3)
                    .curveToConstantSync(Poses.FarPoses.spikeCollectCP.mirror(fieldLength), Poses.FarPoses.spikeCollectPose.mirror(fieldLength),Math.PI)
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
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(() -> LLMath.scan(follower))
                    .addTask(() -> SmartCollectTimer = new ElapsedTime())
                    .addTask(() -> follower.getCurrentTValue() > 0.5)
                    .addTask(() -> !follower.isBusy() || Math.abs(Odo.odo.getVelX(DistanceUnit.METER)) < 0.5)
                    .addTask(() -> follower.turn(Math.toRadians(30), false))
                    .waitSeconds(0.05)
                    .addTask(() -> follower.turn(Math.toRadians(30), true))
                    .waitSeconds(0.05)
                    .addTask(()->follower.setMaxPower(1))
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
        offset = 3.5;
        velocity = Shooter.velocityFAR;
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
        alliance = Alliance.BLUE;
    }
}