package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootAutoClose;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.util.wrappers.NotVibeCoded;

//@Autonomous(name = "Close Blue", group = "Auto")
@NotVibeCoded
public class CloseBlue extends Auto {


    public class ShootPreload implements Task {
        Command command;

        public ShootPreload(){
            command = new Command(follower);
            command.lineToTangentialASync(Poses.ClosePoses.shootPose.mirror(144))
                    .waitSeconds(0.05)
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
                    .curveToConstantAsync(Poses.ClosePoses.collectSecondSpikeCP.mirror(144), Poses.ClosePoses.collectSecondSpike.mirror(144), Math.PI)
                    .addTask(Intake::stop)
                    .addTask(Storage::stop)
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose.mirror(144))
                    .waitSeconds(0.05)
                    .addTask(new ShootAutoClose());
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
                    .curveToTangentialToConstantSync(Poses.ClosePoses.gateCollectCP.mirror(144), Poses.ClosePoses.gateCollect2.mirror(144), 0.55)
                    .addTask(() -> follower.getCurrentTValue() > 0.8)
                    .addTask(() -> time = new ElapsedTime())
                    .addTask(() -> (Storage.status.equals(Storage.Status.STORAGEFULL) || time.milliseconds() > 1200))
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose.mirror(144))
                    .waitSeconds(0.01)
                    .addTask(new ShootAutoClose());
        }

        @Override
        public boolean Run() {
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
                    .lineToTangentialASync(Poses.ClosePoses.collectFirstSpike.mirror())
                    .lineToConstantAsync(Poses.ClosePoses.shootPoseLeave.mirror(), Math.toRadians(180))
                    .waitSeconds(0.01)
                    .addTask(new ShootAutoClose());
        }

        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }

    @Override
    public void buildAuto() {
        pos = 0.834;
        velocity = 1600;
        follower.setStartingPose(Poses.ClosePoses.startingPose.mirror(144));
        auto.addTask(new ShootPreload())
                .addTask(new ShootSecondSpike())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new ShootFirstSpike());


    }

    @Override
    public void setAlliance() {
        alliance = Alliance.RED;
    }
}