package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootAutoClose;
public class BlueSolo extends Auto{

    double fieldLength = 141;

    public class ShootPreload implements Task {
        Command command;

        public ShootPreload(){
            command = new Command(follower);
            command.addTask(() -> Shooter.shooterEnabled = true)
                    .lineToConstantSync(Poses.ClosePoses.shootPose.mirror(fieldLength), Math.toRadians(270))
                    .addTask(() -> follower.getCurrentTValue() > 0.75)
                    .addTask(() -> follower.setMaxPower(0.5))
                    .addTask(new ShootAutoClose());

        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }

    public class ShootSecondSpike implements Task {
        Command command;
        public ShootSecondSpike() {
            command = new Command(follower);
            command.addTask(Intake::start)
                    .addTask(Storage::start)
                    .addTask(() -> follower.setMaxPower(1))
                    .curveToConstantAsync(Poses.ClosePoses.collectSecondSpikeCP.mirror(fieldLength), Poses.ClosePoses.collectSecondSpike.mirror(fieldLength), 180)
                    .addTask(Storage::stop)
                    .lineToTangentialSyncReversed(Poses.ClosePoses.shootPose.mirror(fieldLength))
                    .addTask(() -> follower.getCurrentTValue() > 0.5)
                    .addTask(() -> follower.setMaxPower(0.5))
                    .addTask(()->follower.getCurrentTValue()>0.6)
                    .addTask(new ShootAutoClose())
                    .addTask(() -> follower.setMaxPower(1));
        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }

    public class GateCycle implements Task {
        Command command;
        ElapsedTime time;
        public GateCycle() {
            command = new Command(follower);
            command.addTask(Intake::start)
                    .addTask(Storage::start)
                    .curveToTangentialToConstantSync(Poses.ClosePoses.gateCollectCP.mirror(fieldLength),Poses.ClosePoses.gateCollect.mirror(fieldLength),0.4)/// ////////
                    .addTask(() -> follower.getCurrentTValue() >= 0.5)/// ///////
                    .addTask(() -> follower.setMaxPower(0.8))/// ///////
                    .addTask(() -> time = new ElapsedTime())
                    .addTask(() -> (Storage.status.equals(Storage.Status.STORAGEFULL) || time.milliseconds() > 1900))
                    .addTask(() -> follower.setMaxPower(1))
                    .lineToTangentialSyncReversed(Poses.ClosePoses.shootPose.mirror(fieldLength))
                    .addTask(() -> follower.getCurrentTValue() > 0.5)
                    .addTask(() -> follower.setMaxPower(0.5))
                    .addTask(()->follower.getCurrentTValue()>0.6)
                    .addTask(new ShootAutoClose())
                    .addTask(() -> follower.setMaxPower(1));
        }

        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }
    public class ShootThirdSpike implements Task{
        Command command;

        public ShootThirdSpike(){
            command = new Command(follower);
            command.addTask(Intake::start)
                    .addTask(Storage::start)
                    .curveToTangentialToConstantAsync(Poses.ClosePoses.collectThirdSpikeCP.mirror(fieldLength), Poses.ClosePoses.collectThirdSpike.mirror(fieldLength), 0.7)
                    .lineToTangentialSyncReversed(Poses.ClosePoses.shootPose.mirror(fieldLength))
                    .addTask(() -> follower.getCurrentTValue() > 0.7)
                    .addTask(() -> follower.setMaxPower(0.5))
                    .addTask(()->follower.getCurrentTValue()>0.8)
                    .addTask(new ShootAutoClose());
        }

        @Override
        public boolean Run(){
            command.update();
            return command.done();
        }
    }
    public class ShootFirstSpike implements Task {
        Command command;
        public ShootFirstSpike() {
            command = new Command(follower);
            command .addTask(Intake::start)
                    .addTask(Storage::start)
                    .lineToTangentialASync(Poses.ClosePoses.collectFirstSpike.mirror(fieldLength))
                    .lineToTangentialSyncReversed(Poses.ClosePoses.shootPose.mirror(fieldLength))
                    .addTask(() -> follower.getCurrentTValue() > 0.6)
                    .addTask(() -> follower.setMaxPower(0.5))
                    .addTask(()->follower.getCurrentTValue()>0.7)
                    .addTask(new ShootAutoClose());
        }

        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }

    public class Park implements Task {
        Command command;
        public Park() {
            command = new Command(follower);
            command.addTask(() -> follower.setMaxPower(0.6))
                    .lineToConstantAsync(Poses.ClosePoses.parkingSpotdeHandicapati.mirror(fieldLength), Math.toRadians(180));
        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }
    public static double inc=0.0052;

    @Override
    public void buildAuto() {
        pos = 0.834+inc;
        velocity = 1600;
        follower.setStartingPose(Poses.ClosePoses.startingPose.mirror(fieldLength));
        auto.addTask(new ShootPreload())
                .addTask(new ShootSecondSpike())
                .addTask(new GateCycle())
                .addTask(new ShootFirstSpike())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new ShootThirdSpike())
                .addTask(new Park());


    }

    @Override
    public void setAlliance() {
        alliance = Alliance.BLUE;
    }
}
