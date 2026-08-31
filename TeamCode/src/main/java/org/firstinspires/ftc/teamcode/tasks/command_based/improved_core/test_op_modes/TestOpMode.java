package org.firstinspires.ftc.teamcode.tasks.command_based.improved_core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.auto_commands.LineToTangentialAsync;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.test_commands.TestCommand;
import org.firstinspires.ftc.teamcode.tasks.command_based.improved_core.test_commands.TestCommand2;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@TeleOp
public class TestOpMode extends BestOpMode {
    @Override
    public void on_start(){};

    @Override
    public void initialize(){
        Constants.createFollower(hardwareMap);
        TestCommand.createCommand();
        TestCommand2.createCommand();
        gamepadEx1.getButton(BestGamepad.Buttons.CROSS)
                .whenPressed(TestCommand.getCommand(),
                        TestCommand.getCommand(),
                        TestCommand.getCommand(),
                        TestCommand2.getCommand());
        gamepadEx1.getButton(BestGamepad.Buttons.CIRCLE)
                .whenPressed(TestCommand2.getCommand(), false);
        waitForStart();
    }

    @Override
    public void initialize_loop(){}
    @Override
    public void active_loop(){}
    @Override
    public void end(){}
}
