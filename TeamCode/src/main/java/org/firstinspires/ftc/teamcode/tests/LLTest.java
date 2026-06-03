package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

public class LLTest extends LinearOpMode {
    Robot r;
    Pose p;
    @Override
    public void runOpMode() throws InterruptedException{
        r = new Robot(hardwareMap, telemetry, Alliance.RED);
        r.initializeRest();
        r.setOpModeType(Robot.OpModeType.TELEOP);
        p = new Pose();
        //setez cv in odometrie plm
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.crossWasPressed()){
                p = LLMath.test();
            }
            telemetry.addData("ball pose x: ", p.getX());
            telemetry.addData("ball pose y: ", p.getY());
            telemetry.update();
            r.update();
        }
    }
}
