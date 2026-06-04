package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.Scheduler;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

public abstract class BetterOpMode extends LinearOpMode {
    public abstract void initialize();
    public abstract void initialize_loop();
    public abstract void on_start();
    public abstract void active_loop();
    public abstract void end();
    public BetterGamepad gamepadEx1, gamepadEx2;
    protected ElapsedTime opModeTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        opModeTimer= new ElapsedTime();
        gamepadEx1= new BetterGamepad(gamepad1);
        gamepadEx2= new BetterGamepad(gamepad2);
        initialize();
        while (opModeInInit()) {
            initialize_loop();
            gamepadEx1.update();
            gamepadEx2.update();
            Scheduler.update();
        }

        on_start();
        opModeTimer= new ElapsedTime();
        while (opModeIsActive()){
            active_loop();
            gamepadEx1.update();
            gamepadEx2.update();
            Scheduler.update();
        }
        end();

    }
}