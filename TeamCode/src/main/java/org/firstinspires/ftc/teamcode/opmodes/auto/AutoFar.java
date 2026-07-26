package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
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
    public void setFlwX(double x){
        flwX = x;
    }
    public abstract void buildAuto();
    public abstract void setAlliance();
    public Robot robot;
    public double offset = 0, flwX;
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
        buildAuto();
    }

    @Override
    public void initialize_loop() {
        follower.update();
        Pose robotPose= follower.getPose();
        Blocker.block();
        Shooter.hoodServo.setPosition(Shooter.hoodFAR);
        //Turret.setNeutralPosition(0.1895);
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.addData("flwX", flwX);
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
        Turret.turrStatus = Turret.TurretStatus.ACTIVE;
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.addData("velo",Shooter.motor1.getVelocity());
        telemetry.addData("dist" , Turret.virtualDist);
        telemetry.addData("targetVel", Shooter.velocityFAR);
        telemetry.addData("cond", Math.abs(Shooter.motor1.getVelocity()) > Shooter.velocityFAR - ShootFar.treshhold);
        telemetry.addData("alliance", alliance);
        telemetry.addData("optimal pose" , LLMath.optimalPose);
        telemetry.addData("transferPower" , Storage.transfer.getPower());
        telemetry.addData("vel", Robot.odo.getVelX(DistanceUnit.METER));
        if(LLMath.results != null) telemetry.addData("LLMath results empty?", LLMath.results.isEmpty());
        telemetry.update();
        Shooter.shooterEnabled = true;
        Turret.updateAuto(follower, offset);
        Shooter.AutoUpdate(Shooter.velocityFAR, Shooter.hoodFAR);
        Storage.updateAuto();
    }

    @Override
    public void end() {

    }
}