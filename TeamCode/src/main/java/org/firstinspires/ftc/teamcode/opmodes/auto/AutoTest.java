package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.graphics.SweepGradient;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

@Autonomous(name = "patrat sper")
public class AutoTest extends Auto{

    public class Square implements Task {
        Command command;
        public Square() {
            command = new Command(follower);
            command.lineToConstantAsync(Poses.ClosePoses.bottomRight, 0)
                    .addTask(() -> !follower.isBusy())
                    .lineToConstantAsync(Poses.ClosePoses.topRight, Math.toRadians(90))
                    .addTask(() -> !follower.isBusy())
                    .lineToConstantAsync(Poses.ClosePoses.topLeft, Math.toRadians(180))
                    .addTask(() -> !follower.isBusy())
                    .lineToConstantAsync(Poses.ClosePoses.testStartingPose, Math.toRadians(270));

        }
        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }
    public class test implements Task {
        Command command;
        public test() {
            command = new Command(follower);
        }

        @Override
        public boolean Run() {
            command.update();
            return command.done();
        }
    }
    @Override
    public void buildAuto(){
        velocity= 950;
        pos= 0.14;
        follower.setStartingPose(Poses.ClosePoses.testStartingPose);
        auto.addTask(new Square());
    }
    @Override
    public void setAlliance(){
        alliance = Alliance.RED;
    }
}
