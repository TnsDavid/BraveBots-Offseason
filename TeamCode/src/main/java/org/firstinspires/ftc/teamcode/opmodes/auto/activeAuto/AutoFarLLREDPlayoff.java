package org.firstinspires.ftc.teamcode.opmodes.auto.activeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses.AutoFar;
import org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses.Poses;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.command_types.ThreadedTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

@Autonomous(name = "Far Red", group = "Auto Far", preselectTeleOp = "TeleOp Red")

public class AutoFarLLREDPlayoff extends AutoFar {
    public class HPCollect implements Task {
        Command s;
        public HPCollect() {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .lineToConstantSync(Poses.FarPoses.HPCollectPose, Poses.FarPoses.HPCollectPose.getHeading())
                    .addTask(()->follower.getCurrentTValue()>0.75)
                    .addTask(() -> follower.setMaxPower(0.7))
                    .addTask(() -> !follower.isBusy())
                    .addTask(() -> follower.turn(Math.toRadians(20), false))
                    .waitSeconds(0.1)
                    .addTask(() -> follower.turn(Math.toRadians(25), true))
                    .waitSeconds(0.07)
                    .addTask(()->follower.setMaxPower(1))
                    .lineToConstantSync(Poses.FarPoses.shootPose, Poses.FarPoses.shootPose.getHeading())
                    .addTask(()->follower.getCurrentTValue() > 0.8)
                    .addTask(Intake::stop)
                    .addTask(() -> !follower.isBusy())
                    .waitSeconds(0.05)
                    .addTask(new ThreadedTask(  ShootFar::new  ,0.4    ));
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
                    .addTask(new ThreadedTask(ShootFar::new  , 0.4));
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    public class CycleCollect implements Task {
        Command s;
        public CycleCollect(boolean lastcycle) {
            s = new Command(follower)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(() -> LLMath.scan(follower))
                    .addTask(() -> follower.getCurrentTValue() > 0.5)
                    .addTask(() -> !follower.isBusy() || Odo.odo.getVelX(DistanceUnit.METER) < 0.5)
                    .addTask(()->follower.setMaxPower(1))
                    .lineToTangentialToConstantSync(Poses.FarPoses.shootPoseScan, 0.7, true)
                    .addTask(() -> follower.getCurrentTValue() > 0.5)
                    .addTask(Intake::stop)
                    .addTask(() -> !follower.isBusy())
                    .addTask(lastcycle ? new ThreadedTask(ShootFar::new, 0.3) : new ShootFar());
        }

        @Override
        public boolean Run(){
            s.update();
            return s.done();
        }
    }

    public class PLEACA_BA_DE_ACOLO implements Task{
        Command s;

        public PLEACA_BA_DE_ACOLO() {
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
        offset = -1.5;
        velocity = 1080;
        pos = 0.5792;
        auto.addTask(new ShootFar())
                .addTask(new HPCollect())
                .addTask(new SpikeCollect())
                .addTask(new CycleCollect(false))
                .addTask(new CycleCollect(false))
                .addTask(new CycleCollect(false))
                .addTask(new CycleCollect(false))
                .addTask(new CycleCollect(false))
                .addTask(new CycleCollect(false));
    }

    @Override
    public void setAlliance(){
        alliance = Alliance.RED;
    }
}