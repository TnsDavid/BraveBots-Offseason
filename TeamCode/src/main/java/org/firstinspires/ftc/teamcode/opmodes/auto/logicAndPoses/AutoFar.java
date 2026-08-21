package org.firstinspires.ftc.teamcode.opmodes.auto.logicAndPoses;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Globals;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
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
        follower = Constants.createFollower(hardwareMap);
        LLMath.init(hardwareMap);

        turretpos= Robot.alliance==Alliance.RED? .1895 : 0.81;
        Turret.setNeutralPosition(turretpos);

        auto = new Command(follower);
        pos = Shooter.hoodFAR;
        Shooter.shooterEnabled = true;
        Shooter.velocity =Shooter.velocityFAR;

        buildAuto();
    }

    public static Pose robotPose = new Pose();
    @Override
    public void initialize_loop() {
        follower.update();
        Pose robotPose= follower.getPose();
        Blocker.block();
        Shooter.hoodServo.setPosition(Shooter.hoodFAR);
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

        robotPose= follower.getPose();
        Globals.lastPose = robotPose;
        Turret.turrStatus = Turret.TurretStatus.ACTIVE;

        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.addData("velo",Shooter.motor1.getVelocity());
        telemetry.update();

        robot.update();
        follower.update();
        auto.update();
        Shooter.shooterEnabled = true;
        Turret.updateAuto(follower, offset);
        Shooter.AutoUpdate(velocity, pos);
        Storage.updateAuto();
    }

    @Override
    public void end() {
            Globals.lastPose = robotPose;
    }
}