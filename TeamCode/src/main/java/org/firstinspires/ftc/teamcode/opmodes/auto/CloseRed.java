package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.app.usage.StorageStats;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.auto_task.ShootAutoClose;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.util.NUMAISUPORT;

@Autonomous(name = "Close Red", group = "Auto")
@NUMAISUPORT
public class CloseRed extends Auto {


    public class ShootPreload implements Task {
        Command command;

        public ShootPreload(){
            command = new Command(follower);
            command.lineToConstantAsync(Poses.ClosePoses.shootPose, Math.toRadians(270))
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
                    .curveToLinearAsync(Poses.ClosePoses.collectSecondSpikeCP, Poses.ClosePoses.collectSecondSpike)
                    .waitSeconds(0.15)
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
                    .lineToLinearAsync(Poses.ClosePoses.gateCollect)
                    .addTask(() -> time = new ElapsedTime())
                    //.addTask(Intake::stop)
                    .addTask(() -> (Storage.status.equals(Storage.Status.STORAGEFULL) || time.milliseconds() > 1100))
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose)
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
        pos = 0.8128;
        velocity = 1680;
        follower.setStartingPose(Poses.ClosePoses.startingPose);
        auto.addTask(new ShootPreload())
                .addTask(new ShootSecondSpike())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle())
                .addTask(new GateCycle());


    }

    @Override
    public void setAlliance() {
        alliance = Alliance.RED;
    }
}