package org.firstinspires.ftc.teamcode.util.wrappers;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tasks.core.Scheduler;

public abstract class BetterOpMode extends LinearOpMode {
    public abstract void initialize();
    public abstract void initializeloop();
    public abstract void onstart();
    public abstract void activeloop();
    public abstract void end();

    public BetterGamepad gpEx1, gpEx2;
    public ElapsedTime opModeTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        opModeTimer = new ElapsedTime();
        gpEx1 = new BetterGamepad(gamepad1);
        gpEx2 = new BetterGamepad(gamepad2);
        initialize();
        while(opModeInInit()){
            initializeloop();
            gpEx1.update();
            gpEx2.update();
            Scheduler.update();
        }
        onstart();
        opModeTimer = new ElapsedTime();
        while(opModeIsActive()){
            activeloop();
            gpEx1.update();
            gpEx2.update();
            Scheduler.update();
        }

        end();
    }
}
