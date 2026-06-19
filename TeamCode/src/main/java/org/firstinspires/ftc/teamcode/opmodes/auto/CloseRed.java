package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.auto_task.ShootAutoClose;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

@Autonomous(name = "Close Red", group = "Auto")

public class CloseRed extends Auto {


    public class ShootPreload implements Task {
        Command command;

        public ShootPreload(){
            command = new Command(follower);
            command.lineToConstantAsync(Poses.ClosePoses.shootPose, Math.toRadians(270))
                    .waitSeconds(0.1)
                    //.addTask(()->follower)
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
                    .curveToConstantAsync(Poses.ClosePoses.collectSecondSpikeCP, Poses.ClosePoses.collectSecondSpike, 0)
                    .addTask(Intake::stop)
                    .addTask(Storage::stop)
                    .lineToLinearAsync(Poses.ClosePoses.shootPose)
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
                    .lineToLinearSync(Poses.ClosePoses.gateCollect)
                    .addTask(() -> follower.getCurrentTValue() >= 0.8)
                    .addTask(() -> follower.setMaxPower(0.7))
                    .addTask(() -> time = new ElapsedTime())
                    .addTask(() -> (Storage.status.equals(Storage.Status.STORAGEFULL) || time.milliseconds() > 1100))
                    .addTask(() -> follower.setMaxPower(1))
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose)
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
            command.lineToLinearAsync(Poses.ClosePoses.breakPoint)
                    .addTask(Intake::start)
                    .addTask(Storage::start)
                    .lineToTangentialASync(Poses.ClosePoses.collectFirstSpike)
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose)
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
            command.lineToConstantAsync(Poses.ClosePoses.parkingSpotdeHandicapati, Math.toRadians(180));
        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }

    @Override
    public void buildAuto() {
        pos = 0.823;
        velocity = 1600;
        follower.setStartingPose(Poses.ClosePoses.startingPose);
        auto.addTask(new ShootPreload())
                .addTask(new ShootSecondSpike())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new ShootFirstSpike())
                .addTask(new Park());


    }

    @Override
    public void setAlliance() {
        alliance = Alliance.RED;
    }
}