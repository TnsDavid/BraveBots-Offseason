package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.globals.Robot;

public abstract class BestOpMode extends LinearOpMode {
    public abstract void initialize();
    public abstract void initialize_loop();
    public abstract void on_start();
    public abstract void active_loop();
    public abstract void end();
    public BestGamepad gamepadEx1, gamepadEx2;
    protected ElapsedTime opModeTimer;
    public BetterScheduler scheduler;

    @Override
    public void runOpMode() throws InterruptedException {
        opModeTimer= new ElapsedTime();
        scheduler = new BetterScheduler();
        gamepadEx1= new BestGamepad(gamepad1, scheduler);
        gamepadEx2= new BestGamepad(gamepad2, scheduler);
        scheduler.schedule(BetterTask.createInfinite(() -> {
            gamepadEx1.update();
            gamepadEx2.update();
        }));
        initialize();
        while (opModeInInit()) {
            initialize_loop();
            scheduler.update();
        }

        on_start();
        opModeTimer= new ElapsedTime();
        while (opModeIsActive()){
            active_loop();
            scheduler.update();
        }
        end();
    }
}