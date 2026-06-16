package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;

public class CloseRed extends Auto {


    public class ShootPreload implements Task {
        Command command;

        public ShootPreload(){
            command = new Command(follower)
                    .lineToTangentialASync(Poses.ClosePoses.shootPose)
                    .addTask(new Shoot());

        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }

    public class ShootSecondSpike implements Task{

        Command command;

        public ShootSecondSpike(){
            command =new Command(follower)
                    .addTask(Intake::toggle)
                    .lineToLinearAsync(Poses.ClosePoses.ControlPointSecondSpike)
                    .lineToTangentialASync(Poses.ClosePoses.CollectSecondSpike)
                    .addTask(Intake::stop)
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose)
                    .addTask(new Shoot());
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
            command = new Command(follower)
                    .lineToTangentialASync(Poses.ClosePoses.collectFirstSpike)
                    .addTask(Intake::stop)
                    .waitSeconds(0.2)
                    .lineToTangentialAsyncReversed(Poses.ClosePoses.shootPose)
                    .addTask(new Shoot());
        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }
    @Override
    public void buildAuto() {
        velocity = 1650;
        pos = 0.7689;
        follower.setStartingPose(Poses.ClosePoses.startingPose);
        auto.addTask(new ShootPreload())
                .addTask(new ShootSecondSpike());
    }

    @Override
    public void setAlliance() {
        alliance = Alliance.RED;
    }
}
