package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;

import java.util.LinkedList;

public abstract class Auto extends BetterOpMode {

    protected Command auto;
    protected Alliance alliance;
    protected Follower follower;
    protected double velocity, pos;
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
        Turret.setNeutralPosition(0.5);
        follower= Constants.createFollower(hardwareMap);
        auto= new Command(follower);
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
        robot.update();
        follower.update();
        auto.update();
        Pose robotPose= follower.getPose();
        Turret.updateAuto(new Pose(robotPose.getX()*0.0254, robotPose.getY()*0.0254, robotPose.getHeading()) , follower);
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.addData("alliance", alliance);
        telemetry.addData("virtual dist" , Turret.virtualDist);
        telemetry.addData("velocity" , Shooter.motor1.getVelocity());
        telemetry.update();
        Intake.update();
        Storage.update();
        Shooter.setVelocity(velocity);
        Shooter.servo.setPosition(pos);
        Shooter.motor1.update();
    }

    @Override
    public void end() {

    }
}