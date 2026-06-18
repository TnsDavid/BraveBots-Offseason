package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.math.LLMath;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;

public abstract class AutoFar extends BetterOpMode {

    protected Command auto;
    protected Alliance alliance;
    protected Follower follower;
    protected double velocity, pos;
    protected double offsetAuto;
    public abstract void buildAuto();
    public abstract void setAlliance();
    public Robot robot;

    @Override
    public void initialize() {
        setAlliance();
        robot= new Robot(hardwareMap, telemetry, alliance)
                .setPlayStyle(Robot.PlayStyle.FAR)
                .setOpModeType(Robot.OpModeType.AUTO)
                .initialize();
        LLMath.init(hardwareMap);
        Turret.setNeutralPosition(0.5);
        follower= Constants.createFollower(hardwareMap);
        auto= new Command(follower);
        pos = Shooter.hoodFAR;
        Blocker.block();
        Shooter.shooterEnabled = true;
        Shooter.velocity =Shooter.velocityFAR;
        Shooter.servo.setPosition(Shooter.hoodFAR);
        offsetAuto = Robot.a == Alliance.RED ? -3.5 : 5;
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
    public void on_start() {}

    @Override
    public void active_loop() {
        robot.update();
        follower.update();
        auto.update();
        Pose robotPose= follower.getPose();
        Turret.updateAuto(follower, offsetAuto);
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.addData("velo",Shooter.motor1.getVelocity());
        telemetry.addData("targetVel", Shooter.velocityFAR);
        telemetry.addData("cond", Math.abs(Shooter.motor1.getVelocity()) > Shooter.velocityFAR - ShootFar.treshhold);
        telemetry.addData("alliance", alliance);
        //telemetry.addData("optimal pose", LLMath.optimalPose);
        telemetry.update();

        Shooter.shooterEnabled = true;
        Shooter.setVelocity(Shooter.velocityFAR);
        Shooter.motor1.update();
        Storage.update();
        //Intake.update();
    }

    @Override
    public void end() {

    }
}