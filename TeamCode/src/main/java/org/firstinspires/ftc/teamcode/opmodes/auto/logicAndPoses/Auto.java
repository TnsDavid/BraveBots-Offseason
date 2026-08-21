package org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;

public abstract class Auto extends BetterOpMode {

    protected Command auto;
    protected Alliance alliance;
    protected Follower follower;
    protected double velocity, pos;
    long now, last, dt;
    public abstract void buildAuto();
    public abstract void setAlliance();
    public Robot robot;
    public double offset, flwX;

    @Override
    public void initialize() {
        setAlliance();
        robot = new Robot(hardwareMap, telemetry, alliance)
                .setOpModeType(Robot.OpModeType.AUTO)
                .setPlayStyle(Robot.PlayStyle.CLOSE)
                .initialize();
        follower= Constants.createFollower(hardwareMap);

        Storage.status = Storage.Status.EMPTY;
        Shooter.shooterEnabled = false;
        auto = new Command(follower);
        offset = alliance == Alliance.RED ? -3 : -1.5;
        Shooter.offset = -50;

        buildAuto();
    }

    @Override
    public void initialize_loop() {
        follower.update();
        Pose robotPose= follower.getPose();

        Shooter.hoodServo.setPosition(pos);
        Blocker.block();

        telemetry.addData("status", Storage.status);
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

        Pose robotPose= follower.getPose();
        Turret.turrStatus = Turret.TurretStatus.ACTIVE;

        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;

        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.update();

        robot.update();
        follower.update();
        auto.update();
        Shooter.AutoUpdate(velocity, pos);
        Turret.updateAuto(follower , offset);
        Storage.update2();
    }

    @Override
    public void end() {

    }
}