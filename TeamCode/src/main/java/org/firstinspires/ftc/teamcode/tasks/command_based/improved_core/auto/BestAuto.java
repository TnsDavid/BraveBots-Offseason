package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.auto_commands.ExecutionMode;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.infinite_commands.AutoUpdate;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;

public abstract class BestAuto extends BetterOpMode {
    protected Alliance alliance;
    protected Follower follower;
    protected BetterScheduler auto;
    long now, last, dt;
    public abstract void buildAuto();
    public abstract void setAlliance();
    public Robot robot;

    @Override
    public void initialize() {
        setAlliance();
        robot= new Robot(hardwareMap, telemetry, alliance)
                .setOpModeType(Robot.OpModeType.AUTO)
                .setPlayStyle(Robot.PlayStyle.CLOSE)
                .initialize();
        sleep(500);
        follower= Constants.createFollower(hardwareMap);
        auto.schedule(BetterTask.createInfinite(() -> {
            follower.update();
            robot.update();
        }));
        buildAuto();
    }

    @Override
    public void initialize_loop() {
        follower.update();
        Pose robotPose= follower.getPose();
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.update();
    }

    @Override
    public void on_start() {
    }

    @Override
    public void active_loop() {
        auto.update();

        Pose robotPose= follower.getPose();
        now = System.nanoTime();
        dt = now - last;
        last = now;

        telemetry.addData("hz", 1e9 / dt);
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.update();
    }

    @Override
    public void end() {

    }
}