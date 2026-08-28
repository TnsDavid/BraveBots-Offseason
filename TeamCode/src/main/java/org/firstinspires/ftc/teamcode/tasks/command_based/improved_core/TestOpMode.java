package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.test_commands.TestCommand;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.test_commands.TestCommand2;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@TeleOp
public class TestOpMode extends LinearOpMode {
    BetterScheduler scheduler;
    Follower follower;
    BestGamepad gamepadEx1, gamepadEx2;

    @Override
    public void runOpMode() throws InterruptedException{
        Constants.createFollower(hardwareMap);
        scheduler = new BetterScheduler(follower);
        gamepadEx1= new BestGamepad(gamepad1, scheduler);
        gamepadEx2= new BestGamepad(gamepad2, scheduler);
        TestCommand.createCommand();
        TestCommand2.createCommand();
        gamepadEx1.getButton(BestGamepad.Buttons.CROSS)
                .whenPressed(TestCommand.getCommand(), true);
        gamepadEx1.getButton(BestGamepad.Buttons.CIRCLE)
                .whenPressed(TestCommand2.getCommand(), false);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.crossWasPressed()){
                scheduler.schedule(TestCommand.getCommand())
                        .schedule(TestCommand.getCommand())
                        .schedule(TestCommand.getCommand())
                        .lineToTangentialAsync(new Pose(20, 20));
            }
            if(gamepad1.squareWasPressed()) scheduler.schedule(TestCommand2.getCommand());
            gamepadEx1.update();
            gamepadEx2.update();
            scheduler.update();
        }
    }
}
