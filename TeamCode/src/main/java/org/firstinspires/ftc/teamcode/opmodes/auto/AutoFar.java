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
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootFar;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

public abstract class AutoFar extends BetterOpMode {

    protected Command auto;
    protected Alliance alliance;
    protected Follower follower;
    protected double velocity, pos;
    public abstract void buildAuto();
    public abstract void setAlliance();
    public Robot robot;
    public double offset = -3.5;
    public double turretpos;

    @Override
    public void initialize() {
        setAlliance();
        robot= new Robot(hardwareMap, telemetry, alliance)
                .setPlayStyle(Robot.PlayStyle.FAR)
                .setOpModeType(Robot.OpModeType.AUTO)
                .initialize();
        sleep(500);
        turretpos= Robot.alliance==Alliance.RED? .1895 : 0.81;
        Turret.setNeutralPosition(turretpos);
        LLMath.init(hardwareMap);
        follower= Constants.createFollower(hardwareMap);
        auto= new Command(follower);
        pos = Shooter.hoodFAR;
        Shooter.shooterEnabled = true;
        Shooter.velocity =Shooter.velocityFAR;
        Shooter.hoodServo.setPosition(Shooter.hoodFAR);
        buildAuto();
    }

    @Override
    public void initialize_loop() {
        follower.update();
        Pose robotPose= follower.getPose();
        //Turret.setNeutralPosition(0.1895);
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
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.addData("velo",Shooter.motor1.getVelocity());
        telemetry.addData("targetVel", Shooter.velocityFAR);
        telemetry.addData("cond", Math.abs(Shooter.motor1.getVelocity()) > Shooter.velocityFAR - ShootFar.treshhold);
        telemetry.addData("alliance", alliance);
        telemetry.update();
        Shooter.shooterEnabled = true;
        Shooter.hoodServo.setPosition(Shooter.hoodFAR);
        Shooter.tuningUpdate(Shooter.velocityFAR);

        Storage.update2();
        Intake.update();
    }

    @Override
    public void end() {

    }
}